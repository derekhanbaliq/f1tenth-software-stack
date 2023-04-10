import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import numpy as np

import torch
import torch.nn as nn
import time

import FrEIA.framework as Ff
import FrEIA.modules as Fm
from FrEIA.framework import InputNode, OutputNode, Node, ReversibleGraphNet, ConditionNode
from FrEIA.modules import GLOWCouplingBlock, PermuteRandom, RNVPCouplingBlock

N_DIM = 60
COND_DIM = 6
COND_OUT_DIM = 12
USE_MEAN = 0
device = torch.device('cuda')
# device = torch.device('cpu')

class PositionalEncoding_torch():
    def __init__(self, L):
        self.L = L
        self.val_list = []
        for l in range(L):
            self.val_list.append(2.0 ** l)
        self.val_list = torch.Tensor(self.val_list)[None, :].to(device)
        self.pi = torch.Tensor([3.14159265358979323846]).to(device)
        
    def encode(self, x):
        return torch.sin(x * self.val_list * self.pi), torch.cos(x * self.val_list * self.pi)
    
    def encode_even(self, x):
        return torch.sin(x * self.val_list * self.pi * 2), torch.cos(x * self.val_list * self.pi * 2)
    
    def batch_encode(self, batch):
        batch_encoded_list = []
        for ind in range(3):
            if ind == 2:
                encoded_ = self.encode_even(batch[:, ind, None])
            else:
                encoded_ = self.encode(batch[:, ind, None])
            batch_encoded_list.append(encoded_[0])
            batch_encoded_list.append(encoded_[1])
        batch_encoded = torch.stack(batch_encoded_list)
        batch_encoded = batch_encoded.transpose(0, 1).transpose(1, 2).reshape((batch_encoded.shape[1], self.L * batch_encoded.shape[0]))
        return batch_encoded

    def batch_decode(self, sin_value, cos_value):
        atan2_value = torch.arctan2(sin_value, cos_value) / (self.pi)
        sub_zero_inds = torch.where(atan2_value < 0)
        atan2_value[sub_zero_inds] = atan2_value[sub_zero_inds] + 1
        return atan2_value

    def batch_decode_even(self, sin_value, cos_value):
        atan2_value = torch.arctan2(sin_value, cos_value) / (self.pi/2)
        sub_zero_inds = torch.where(atan2_value < 0)
        atan2_value[sub_zero_inds] = atan2_value[sub_zero_inds] + 1
        atan2_value[torch.where(torch.abs(atan2_value - 1) < 0.001)] = 0
        return atan2_value
    


class PositionalEncoding():
    def __init__(self, L):
        self.L = L
        self.val_list = []
        for l in range(L):
            self.val_list.append(2.0 ** l)
        self.val_list = np.array(self.val_list)

    def encode(self, x):
        return np.sin(self.val_list * np.pi * x), np.cos(self.val_list * np.pi * x)
    
    def encode_even(self, x):
        return np.sin(self.val_list * np.pi * 2 * x), np.cos(self.val_list * np.pi * 2 * x)
    
    def decode(self, sin_value, cos_value):
        atan2_value = np.arctan2(sin_value, cos_value) / (np.pi)
        if np.isscalar(atan2_value) == 1:
            if atan2_value > 0:
                return atan2_value
            else:
                return 1 + atan2_value
        else:
            atan2_value[np.where(atan2_value < 0)] = atan2_value[np.where(atan2_value < 0)] + 1
            return atan2_value
        
    def decode_even(self, sin_value, cos_value):
        atan2_value = np.arctan2(sin_value, cos_value) / np.pi/2
        if np.isscalar(atan2_value) == 1:
            if atan2_value < 0:
                atan2_value = 1 + atan2_value
            if np.abs(atan2_value - 1) < 0.001:
                atan2_value = 0
        else:
            atan2_value[np.where(atan2_value < 0)] = atan2_value[np.where(atan2_value < 0)] + 1
            atan2_value[np.where(np.abs(atan2_value - 1) < 0.001)] = 0
        return atan2_value



class VariationalEncoder(nn.Module):
    def __init__(self):
        super(VariationalEncoder, self).__init__()
        self.linear1 = nn.Linear(270, 270)
        self.linear2 = nn.Linear(270, 54)
        self.linear3 = nn.Linear(270, 54)

    def forward(self, x, random_nums):
        x = torch.nn.functional.relu(self.linear1(x))
        mu =  self.linear2(x)
        sigma = torch.exp(self.linear3(x))
        z = mu + sigma * random_nums.to(device)
        # self.kl = (sigma**2 + mu**2 - torch.log(sigma) - 1/2).mean()
        return z
    
class Decoder(nn.Module):
    def __init__(self):
        super(Decoder, self).__init__()
        self.linear1 = nn.Linear(54, 270)
        self.linear2 = nn.Linear(270, 270)
        
    def forward(self, z):
        z = nn.functional.relu(self.linear1(z))
        z = torch.sigmoid(self.linear2(z))
        return z
    
class VariationalAutoencoder(nn.Module):
    def __init__(self):
        super(VariationalAutoencoder, self).__init__()
        self.encoder = VariationalEncoder().to(device)
        self.decoder = Decoder().to(device)
    
    def forward(self, x):
        z = self.encoder(x)
        return self.decoder(z)

class Local_INN(nn.Module):
    def __init__(self):
        super().__init__()
        self.model = self.build_inn()
        
        self.trainable_parameters = [p for p in self.model.parameters() if p.requires_grad]
        for param in self.trainable_parameters:
            param.data = 0.05 * torch.randn_like(param)
        self.cond_net = self.subnet_cond()
        self.vae = VariationalAutoencoder()
        
    def subnet_cond(self):
        return nn.Sequential(nn.Linear(COND_DIM, 256), nn.ReLU(),
                             nn.Linear(256, COND_OUT_DIM))
        
    def build_inn(self):

        def subnet_fc(dim_in, dim_out):
            return nn.Sequential(nn.Linear(dim_in, 1024), nn.ReLU(),
                                    nn.Linear(1024, dim_out))

        nodes = [InputNode(N_DIM, name='input')]
        cond = ConditionNode(COND_OUT_DIM, name='condition')
        for k in range(6):
            nodes.append(Node(nodes[-1],
                                GLOWCouplingBlock,
                                {'subnet_constructor': subnet_fc, 'clamp': 2.0},
                                conditions=cond,
                                name=F'coupling_{k}'))
            nodes.append(Node(nodes[-1],
                                PermuteRandom,
                                {'seed': k},
                                name=F'permute_{k}'))

        nodes.append(OutputNode(nodes[-1], name='output'))
            
        return ReversibleGraphNet(nodes + [cond], verbose=False).to(device)
    
    def forward(self, x, cond):
        return self.model(x, self.cond_net(cond))
    
    def reverse(self, y_rev, cond):
        return self.model(y_rev, self.cond_net(cond), rev=True)
    
TRT_LOGGER = trt.Logger()    
class HostDeviceMem(object):
    def __init__(self, host_mem, device_mem):
        self.host = host_mem
        self.device = device_mem

    def __str__(self):
        return "Host:\n" + str(self.host) + "\nDevice:\n" + str(self.device)

    def __repr__(self):
        return self.__str__()

def allocate_buffers(engine):
    inputs = []
    outputs = []
    bindings = []
    stream = cuda.Stream()
    for binding in engine:
        size = trt.volume(engine.get_binding_shape(binding)) * engine.max_batch_size
        dtype = trt.nptype(engine.get_binding_dtype(binding))
        # Allocate host and device buffers
        host_mem = cuda.pagelocked_empty(size, dtype)
        device_mem = cuda.mem_alloc(host_mem.nbytes)
        # Append the device buffer to device bindings.
        bindings.append(int(device_mem))
        # Append to the appropriate list.
        if engine.binding_is_input(binding):
            inputs.append(HostDeviceMem(host_mem, device_mem))
        else:
            outputs.append(HostDeviceMem(host_mem, device_mem))
    return inputs, outputs, bindings, stream

def do_inference_v2(context, bindings, inputs, outputs, stream):
    # Transfer input data to the GPU.
    [cuda.memcpy_htod_async(inp.device, inp.host, stream) for inp in inputs]
    # Run inference.
    context.execute_async_v2(bindings=bindings, stream_handle=stream.handle)
    # Transfer predictions back from the GPU.
    [cuda.memcpy_dtoh_async(out.host, out.device, stream) for out in outputs]
    # Synchronize the stream
    stream.synchronize()
    # Return only the host outputs.
    return [out.host for out in outputs]

def do_inference(context, bindings, inputs, outputs, stream, batch_size=1):
    # Transfer input data to the GPU.
    [cuda.memcpy_htod_async(inp.device, inp.host, stream) for inp in inputs]
    # Run inference.
    context.execute_async(batch_size=batch_size, bindings=bindings, stream_handle=stream.handle)
    # Transfer predictions back from the GPU.
    [cuda.memcpy_dtoh_async(out.host, out.device, stream) for out in outputs]
    # Synchronize the stream
    stream.synchronize()
    # Return only the host outputs.
    return [out.host for out in outputs]


def get_engine(engine_file_path):
    with open(engine_file_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
        return runtime.deserialize_cuda_engine(f.read())

class Local_INN_Reverse(nn.Module):
    def __init__(self, sample_num):
        super().__init__()
        self.model = Local_INN()
        self.sample_num = sample_num
        
    def forward(self, scan_t, encoded_cond_t, random_nums1, random_nums2):
        sample_num = self.sample_num
        encode_scan = torch.zeros((sample_num, N_DIM)).to(torch.device('cuda'))
        encode_scan[:, :54] = self.model.vae.encoder.forward(scan_t, random_nums1)
        encode_scan[1:, 54:] = random_nums2
        encoded_cond = encoded_cond_t[None].repeat(sample_num, 1).view(-1, COND_DIM)
        encoded_result = self.model.reverse(encode_scan.to(device), encoded_cond.to(device))[0]
        return encoded_result


from utils import ConfigJSON, DataProcessor, DrivableCritic
import numpy as np
class Local_INN_TRT_Runtime():
    def __init__(self, EXP_NAME, use_trt, sample_num) -> None:
        self.data_proc = DataProcessor()
        self.c = ConfigJSON()
        self.c.load_file('models/' + EXP_NAME + '/' + EXP_NAME + '.json')
        self.p_encoding_c = PositionalEncoding(L = 1)
        self.p_encoding = PositionalEncoding(L = 10)
        # self.p_encoding_torch = PositionalEncoding_torch(L = 10)
        self.use_trt = use_trt
        self.sample_num = sample_num

        self.device = torch.device('cuda:0')
        
        if self.use_trt:
            engine_file_path = 'models/' + EXP_NAME + '/' + EXP_NAME + '.trt'
            self.engine = get_engine(engine_file_path)
            self.context = self.engine.create_execution_context()
            self.inputs, self.outputs, self.bindings, self.stream = allocate_buffers(self.engine)
            time.sleep(3) # wait for engine loading
        else:
            self.local_inn_reverse = Local_INN_Reverse(self.sample_num)
            self.local_inn_reverse.model.load_state_dict(torch.load('models/'+EXP_NAME+'/'+EXP_NAME+'_model_best.pt', map_location=self.device))
            self.local_inn_reverse.to(self.device)
            self.local_inn_reverse.eval()
        print('Model loaded.')

    def reverse(self, scan, prev_state):
        sample_num = self.sample_num
        ## normalize scan and sample the latent space
        scan_np = self.data_proc.runtime_normalize(np.array(scan), self.c.d['normalization_laser'])

        ## normalize prev_state
        prev_state_normal = np.array(prev_state).copy()
        prev_state_normal[0] = self.data_proc.runtime_normalize(prev_state[0], self.c.d['normalization_x'])
        prev_state_normal[1] = self.data_proc.runtime_normalize(prev_state[1], self.c.d['normalization_y'])
        prev_state_normal[2] = self.data_proc.runtime_normalize(prev_state[2], self.c.d['normalization_theta'])
        prev_state_normal = np.round(prev_state_normal, decimals=1)
        
        ## encode the prev_state
        encoded_cond = []
        for k in range(3):
            if k == 2:
                sine_part, cosine_part = self.p_encoding_c.encode_even(prev_state_normal[k])
            else:
                sine_part, cosine_part = self.p_encoding_c.encode(prev_state_normal[k])
            encoded_cond.append(sine_part)
            encoded_cond.append(cosine_part)
        encoded_cond = np.array(encoded_cond)
        encoded_cond = encoded_cond.flatten('F')
        
        random_nums1 = np.random.default_rng().normal(size=(sample_num, 54))
        random_nums2 = np.random.default_rng().normal(size=(sample_num-1, 6))
        
        ## infer
        if self.use_trt:
            self.inputs[0].host = np.array(scan_np, dtype=np.float32, order='C')
            self.inputs[1].host = np.array(encoded_cond, dtype=np.float32, order='C')
            self.inputs[2].host = np.array(random_nums1, dtype=np.float32, order='C')
            self.inputs[3].host = np.array(random_nums2, dtype=np.float32, order='C')
            trt_outputs = do_inference_v2(self.context, bindings=self.bindings, inputs=self.inputs, outputs=self.outputs, stream=self.stream)
            encoded_result = np.array(trt_outputs).reshape(sample_num, 60)  
        else:
            scan_t = torch.from_numpy(np.array(scan_np)).type('torch.FloatTensor').to(self.device)
            encoded_cond = torch.from_numpy(encoded_cond).type('torch.FloatTensor').to(self.device)  
            random_nums1 = torch.from_numpy(random_nums1).type('torch.FloatTensor').to(self.device) 
            random_nums2 = torch.from_numpy(random_nums2).type('torch.FloatTensor').to(self.device)      
            encoded_result = self.local_inn_reverse(scan_t, encoded_cond, random_nums1, random_nums2)
            encoded_result = encoded_result.cpu().detach().numpy()
        
        ## decode and de-normalize the results
        results = np.zeros([sample_num, 3])
        results[:, 0] = self.p_encoding.decode(encoded_result[:, 0], encoded_result[:, 1])
        results[:, 1] = self.p_encoding.decode(encoded_result[:, 2], encoded_result[:, 3])
        results[:, 2] = self.p_encoding.decode_even(encoded_result[:, 4], encoded_result[:, 5])
        results[:, 0] = self.data_proc.de_normalize(results[:, 0], self.c.d['normalization_x'])
        results[:, 1] = self.data_proc.de_normalize(results[:, 1], self.c.d['normalization_y'])
        results[:, 2] = self.data_proc.de_normalize(results[:, 2], self.c.d['normalization_theta'])
        
        
        ## find the average
        if USE_MEAN:
            result = np.zeros(3)
            average_angle = np.arctan2(np.mean(np.sin(results[:, 2])), np.mean(np.cos(results[:, 2])))
            if average_angle < 0:
                average_angle += np.pi * 2
            result[2] = average_angle
            result[:2] = np.median(results[:, :2], axis=0)
        else:
            result = results[0]
            
        return result, results
    

