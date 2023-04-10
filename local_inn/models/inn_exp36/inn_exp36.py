import torch
import torch.nn as nn
import numpy as np
from torch.cuda.amp import GradScaler, autocast
# import matplotlib.pyplot as plt
# from torch.profiler import profile, record_function, ProfilerActivity
from os.path import exists

# FrEIA imports
import FrEIA.framework as Ff
import FrEIA.modules as Fm
from FrEIA.framework import InputNode, OutputNode, Node, ReversibleGraphNet, ConditionNode
from FrEIA.modules import GLOWCouplingBlock, PermuteRandom, RNVPCouplingBlock
from utils import ConfigJSON, DataProcessor, mmd_multiscale, DrivableCritic
import time

EXP_NAME = 'inn_exp36'
INSTABILITY_RECOVER = 1
USE_MIX_PRECISION_TRAINING = 0
CONTINUE_TRAINING = 0
TRANSFER_TRAINING = 0
RE_PROCESS_DATA = 1

DATA_USAGE = int(1e5)
BATCHSIZE = int(5e2)
N_DIM = 60
COND_DIM = 6
COND_OUT_DIM = 12
RL = 1e-3

COND_NOISE = [0.2, 0.2, 15 / 180 * np.pi] # m, deg
SCAN_NOISE = 0.005
# SPARKLE_NOISE = 0.01

device = torch.device('cuda:0')

class VariationalEncoder(nn.Module):
    def __init__(self):
        super(VariationalEncoder, self).__init__()
        self.linear1 = nn.Linear(270, 270)
        self.linear2 = nn.Linear(270, 54)
        self.linear3 = nn.Linear(270, 54)
        
        self.N = torch.distributions.Normal(0, 1)
        self.N.loc = self.N.loc.cuda() # hack to get sampling on the GPU
        self.N.scale = self.N.scale.cuda()
        self.kl = 0
    
    def forward(self, x):
        x = torch.nn.functional.relu(self.linear1(x))
        mu =  self.linear2(x)
        sigma = torch.exp(self.linear3(x))
        z = mu + sigma*self.N.sample(mu.shape)
        self.kl = (sigma**2 + mu**2 - torch.log(sigma) - 1/2).mean()
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
        self.encoder = VariationalEncoder()
        self.decoder = Decoder()
    
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
        self.cond_net = self.subnet_cond(COND_OUT_DIM)
        self.vae = VariationalAutoencoder()
        
    def subnet_cond(self, c_out):
        return nn.Sequential(nn.Linear(COND_DIM, 256), nn.ReLU(),
                             nn.Linear(256, COND_OUT_DIM))
        
    def build_inn(self):

        def subnet_fc(c_in, c_out):
            return nn.Sequential(nn.Linear(c_in, 1024), nn.ReLU(),
                                    nn.Linear(1024, c_out))

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
    
class PositionalEncoding_torch():
    def __init__(self, L, device):
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
        # batch_encoded = torch.stack((self.encode(batch[:, 0, None])[0], self.encode(batch[:, 0, None])[1],
        #                              self.encode(batch[:, 1, None])[0], self.encode(batch[:, 1, None])[1],
        #                              self.encode(batch[:, 2, None])[0], self.encode(batch[:, 2, None])[1]))
        batch_encoded_list = []
        for ind in range(3):
            if ind == 2:
                encoded_ = self.encode_even(batch[:, ind, None])
            else:
                encoded_ = self.encode(batch[:, ind, None])
            batch_encoded_list.append(encoded_[0])
            batch_encoded_list.append(encoded_[1])
        batch_encoded = torch.stack(batch_encoded_list)
        # print(batch_encoded.shape)
        # print(batch_encoded)
        batch_encoded = batch_encoded.transpose(0, 1).transpose(1, 2).reshape((batch_encoded.shape[1], self.L * batch_encoded.shape[0]))
        return batch_encoded
    
class F110Dataset(torch.utils.data.Dataset):
    def __init__(self, data):
        self.data = torch.from_numpy(data).type('torch.FloatTensor').to(device)
        # self.data = torch.from_numpy(data).type('torch.FloatTensor')

    def __len__(self):
        return len(self.data)

    def __getitem__(self, index):
        return self.data[index]



def main():
    from trainer import Trainer
    from tqdm import trange
    from torch.utils.tensorboard import SummaryWriter
    from torchinfo import summary
    writer = SummaryWriter('result/tensorboard/' + EXP_NAME)
    
    with open(__file__, "r") as src:
        with open('result/' + EXP_NAME + '.py', "w") as tgt:
            tgt.write(src.read())
            
    if exists('total_data.npy') and not RE_PROCESS_DATA:
        total_data = np.load('total_data.npy')
        data_proc = DataProcessor()
        drivable_critic = DrivableCritic('levine_slam_dark.yaml')
    else:
        data_dir = '/home/lucerna/Documents/DATA/'
        data_dir = 'data/'
        total_data = np.load(data_dir + 'levine360_uni_1e5.npz')['arr_0']
        total_data = total_data[:DATA_USAGE]
        data_proc = DataProcessor()
        total_data[:, 2] = data_proc.two_pi_warp(total_data[:, 2])
        drivable_critic = DrivableCritic('levine_slam_dark.yaml')
        
        c = ConfigJSON()
        normalize_params = drivable_critic.get_normalize_params()
        total_data[:, 0], c.d['normalization_x'] = data_proc.data_normalize(total_data[:, 0])
        total_data[:, 1], c.d['normalization_y'] = data_proc.data_normalize(total_data[:, 1])
        c.d['normalization_theta'] = list(normalize_params[2])
        c.d['normalization_laser'] = list(normalize_params[3])
        c.save_file('result/' + EXP_NAME + '.json')
        c.save_file('train_data.json')
        total_data[:, 3:] = data_proc.runtime_normalize(total_data[:, 3:], normalize_params[3])
        
        ## Augment the data
        def scan_ind_warp(inds, num):
            inds[np.where(inds >= num)] = inds[np.where(inds >= num)] - num
            return inds

        total_scans = total_data[:, 3:]
        print(total_scans.shape)
        scan_len = 360
        augmented_data = []
        for ind_0 in trange(total_data.shape[0]):
            randints = np.concatenate([np.array([0]), np.random.choice(np.arange(10, 270), 9, replace=False)])
            for ind_1 in randints:
                new_scan_inds = scan_ind_warp(np.arange(ind_1 + 45, ind_1 + 270 + 45), scan_len)
                new_angle = (np.pi * 2 / scan_len) * ind_1 + total_data[ind_0, 2]
                new_data = np.concatenate([total_data[ind_0, :2], np.array([new_angle]), total_scans[ind_0, new_scan_inds]])
                augmented_data.append(new_data)
        total_data = np.array(augmented_data)
        total_data[:, 2] = data_proc.two_pi_warp(total_data[:, 2]) # rewarp the theta
        total_data[:, 2] = data_proc.runtime_normalize(total_data[:, 2], normalize_params[2]) # theta is normalized after augmenting
        
        augmented_data = []
        positional_encoding = PositionalEncoding(L = 10)
        for ind_0 in trange(total_data.shape[0]):
            encoded_data = []
            for k in range(3):
                if k == 2:
                    sine_part, cosine_part = positional_encoding.encode_even(total_data[ind_0, k])
                else:
                    sine_part, cosine_part = positional_encoding.encode(total_data[ind_0, k])
                encoded_data.append(sine_part)
                encoded_data.append(cosine_part)
            encoded_data = np.array(encoded_data)
            encoded_data = encoded_data.flatten('F')
            
            new_data = np.concatenate([encoded_data,
                                    total_data[ind_0, 3:], 
                                    total_data[ind_0, :3]])
            augmented_data.append(new_data)
        total_data = np.array(augmented_data)
        
        np.save('total_data.npy', total_data)
    
    print(total_data.shape)
    train_data = total_data
    p_encoding_t = PositionalEncoding_torch(1, device)
    
    cond_noise = np.array(COND_NOISE) # m, rad
    cond_noise[0] /= drivable_critic.x_in_m
    cond_noise[1] /= drivable_critic.y_in_m
    cond_noise[2] /= np.pi * 2
    cond_noise = torch.from_numpy(cond_noise).type('torch.FloatTensor').to(device)

    train_set = F110Dataset(train_data)
    # train_loader = torch.utils.data.DataLoader(dataset=train_set, batch_size=BATCHSIZE, shuffle=True, pin_memory=True, num_workers=5)
    train_loader = torch.utils.data.DataLoader(dataset=train_set, batch_size=BATCHSIZE, shuffle=True, drop_last=False)

    l1_loss = torch.nn.L1Loss()
        
    ### Training
    trainer = Trainer(EXP_NAME, 1000, 0.001, device,
                      RL, [300], 0.05, 'exponential',
                      INSTABILITY_RECOVER, 3, 0.99, USE_MIX_PRECISION_TRAINING)

    model = Local_INN()
    model.to(device)
    if CONTINUE_TRAINING or TRANSFER_TRAINING:
        print('Continue Training')
        model = trainer.continue_train_load(model, path='result/', transfer=TRANSFER_TRAINING)
    
    current_lr = RL
    optimizer = torch.optim.Adam(model.trainable_parameters, lr=current_lr)
    optimizer.add_param_group({"params": model.cond_net.parameters(), "lr": current_lr})
    optimizer.add_param_group({"params": model.vae.encoder.parameters(), "lr": current_lr})
    optimizer.add_param_group({"params": model.vae.decoder.parameters(), "lr": current_lr})
    n_hypo = 20
    epoch_time = 0
    mix_precision = False
    scaler = GradScaler(enabled=mix_precision)
    
    while(not trainer.is_done()):
        epoch = trainer.epoch
        # epoch_info = np.ones(4) * 0.002 # forward_loss, reverse_loss, validataion_reverse_loss, epoch
        epoch_info = np.zeros(4)
        epoch_info_extra = np.zeros(4)
        epoch_info[3] = epoch
        epoch_time_start = time.time()
        
        if USE_MIX_PRECISION_TRAINING:
            if epoch > 0 and mix_precision == False:
                mix_precision = True
                scaler = GradScaler(enabled=mix_precision)
        
        trainer_lr = trainer.get_lr()
        if trainer_lr != current_lr:
            current_lr = trainer_lr
            optimizer.param_groups[0]['lr'] = current_lr
            optimizer.param_groups[1]['lr'] = current_lr
            optimizer.param_groups[2]['lr'] = current_lr
            optimizer.param_groups[3]['lr'] = current_lr
        
        model.train()
        for data in train_loader:
            optimizer.zero_grad()
            with autocast(enabled=mix_precision):
                # data = data.to(device)
                x_hat_gt = data[:, :60]
                cond = data[:, 60+270:60+270+3]
                y_gt = data[:, 60:60+270]
                # y_gt += y_gt * torch.zeros_like(y_gt, device=device).normal_(0., SCAN_NOISE)
                cond += torch.zeros_like(cond, device=device).normal_(0., 1.) * cond_noise
                cond = p_encoding_t.batch_encode(cond.round(decimals=1))
                
                y_hat_vae = torch.zeros_like(x_hat_gt, device=device)
                y_hat_vae[:, :-6] = model.vae.encoder.forward(y_gt)
                y_hat_inn, _ = model(x_hat_gt, cond)
                y_inn = model.vae.decoder.forward(y_hat_inn[:, :-6])
                y_vae = model.vae.decoder.forward(y_hat_vae[:, :-6])
                
                vae_kl_loss = model.vae.encoder.kl * 0.0001
                vae_recon_loss = l1_loss(y_vae, y_gt)
                inn_recon_loss = l1_loss(y_inn, y_gt)
                y_hat_inn_loss = l1_loss(y_hat_inn[:, :-6], y_hat_vae[:, :-6])
                loss_forward = vae_recon_loss + inn_recon_loss + y_hat_inn_loss + vae_kl_loss
                
            
            scaler.scale(loss_forward).backward(retain_graph=True)
            epoch_info[0] += loss_forward.item()

            with autocast(enabled=mix_precision):
                
                # y_hat_vae[:, -6:] = y_hat_inn.detach()[:, -6:]
                y_hat_vae[:, -6:] = 0
                x_hat_0, _ = model.reverse(y_hat_vae, cond)
                loss_reverse = l1_loss(x_hat_0[:, :12], x_hat_gt[:, :12])
                
                batch_size = y_gt.shape[0]
                z_samples = torch.cuda.FloatTensor(n_hypo, batch_size, 6, device=device).normal_(0., 1.)
                y_hat = y_hat_vae[None, :, :54].repeat(n_hypo, 1, 1)
                y_hat_z_samples = torch.cat((y_hat, z_samples), dim=2).view(-1, 60)
                cond = cond[None].repeat(n_hypo, 1, 1).view(-1, COND_DIM)
                x_hat_i = model.reverse(y_hat_z_samples, cond)[0].view(n_hypo, batch_size, 60)
                x_hat_i_loss = torch.mean(torch.min(torch.mean(torch.abs(x_hat_i[:, :, :12] - x_hat_gt[:, :12]), dim=2), dim=0)[0])
                loss_reverse += x_hat_i_loss

                with torch.no_grad():
                    loss_reverse_2 = l1_loss(x_hat_0[:, :6], x_hat_gt[:, :6])
                    loss_reverse_2 += torch.mean(torch.min(torch.mean(torch.abs(x_hat_i[:, :, :6] - x_hat_gt[:, :6]), dim=2), dim=0)[0])

            scaler.scale(loss_reverse).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), 8)

            scaler.step(optimizer)
            scaler.update()
            
            epoch_info[1] += loss_reverse.item()
            epoch_info[2] += loss_reverse_2.item()

            epoch_info_extra[0] += vae_recon_loss.item()
            epoch_info_extra[1] += vae_kl_loss.item()
            epoch_info_extra[2] += y_hat_inn_loss.item()
            epoch_info_extra[3] += inn_recon_loss.item()

            
        epoch_info[:3] /= len(train_loader)
        epoch_info_extra /= len(train_loader)

        model, return_text, mix_precision = trainer.step(model, epoch_info, mix_precision)
        epoch_time = (time.time() - epoch_time_start)
        remaining_time = (trainer.max_epoch - epoch) * epoch_time / 3600
        
        writer.add_scalar("INN/forward", epoch_info[0], epoch)
        writer.add_scalar("INN/reverse", epoch_info[1], epoch)
        writer.add_scalar("INN/reverse_runtime", epoch_info[2], epoch)
        writer.add_scalar("INN/RL", current_lr, epoch)
        writer.add_scalar("INN/y_vae", epoch_info_extra[0], epoch)
        writer.add_scalar("INN/inn_recon", epoch_info_extra[3], epoch)
        writer.add_scalar("INN/vae_kl", epoch_info_extra[1], epoch)
        writer.add_scalar("INN/inn_y", epoch_info_extra[2], epoch)
        writer.add_scalar("INN/remaining(h)", remaining_time, epoch)
        
        print("Epoch {:d}".format(epoch),
            '| f {:.5f}'.format(epoch_info[0]),
            '| r {:.5f}'.format(epoch_info[1]),
            '| r_runtime {:.5f}'.format(epoch_info[2]),
            '| y_vae {:.5f}'.format(epoch_info_extra[0]),
            '| inn_recon {:.5f}'.format(epoch_info_extra[3]),
            '| vae_kl {:.5f}'.format(epoch_info_extra[1]),
            '| inn_y {:.5f}'.format(epoch_info_extra[2]),
            '| remaining(h) {:.1f}'.format(remaining_time), 
            '| ' + return_text)

    writer.flush()


if __name__ == '__main__':
    main()
