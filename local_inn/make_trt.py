import torch
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
from local_inn_trt import *

import sys, os

TRT_LOGGER = trt.Logger()
EXPLICIT_BATCH = 1 << (int)(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)

device = torch.device('cpu')
sample_num = 20
model = Local_INN_Reverse(sample_num)
EXP_NAME = 'inn_exp36'
model.model.load_state_dict(torch.load('models/'+EXP_NAME+'/'+EXP_NAME+'_model_best.pt', map_location=device))
model = model.to(device)
scan_t = torch.zeros(270).to(device)
cond_t = torch.zeros(6).to(device)

random_nums1 = np.random.normal(size=(sample_num, 54))
random_nums1 = torch.from_numpy(random_nums1).type('torch.FloatTensor').to(device)

random_nums2 = np.random.normal(size=(sample_num-1, 6))
random_nums2 = torch.from_numpy(random_nums2).type('torch.FloatTensor').to(device)
dummy_input = (scan_t, cond_t, random_nums1, random_nums2)

torch.onnx.export(model, dummy_input, EXP_NAME + ".onnx", verbose=True, opset_version=13)

def get_engine(onnx_file_path, engine_file_path=""):
    """Attempts to load a serialized engine if available, otherwise builds a new TensorRT engine and saves it."""

    def build_engine():
        """Takes an ONNX file and creates a TensorRT engine to run inference with"""
        with trt.Builder(TRT_LOGGER) as builder, builder.create_network(
            EXPLICIT_BATCH
        ) as network, builder.create_builder_config() as config, trt.OnnxParser(
            network, TRT_LOGGER
        ) as parser, trt.Runtime(
            TRT_LOGGER
        ) as runtime:
            config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 28) # 256 MiB
            # if builder.platform_has_fast_fp16:
            #     config.set_flag(trt.BuilderFlag.FP16)
            
            # Parse model file
            if not os.path.exists(onnx_file_path):
                print(
                    "ONNX file {} not found, please run yolov3_to_onnx.py first to generate it.".format(onnx_file_path)
                )
                exit(0)
                
            print("Loading ONNX file from path {}...".format(onnx_file_path))
            with open(onnx_file_path, "rb") as model:
                print("Beginning ONNX file parsing")
                if not parser.parse(model.read()):
                    print("ERROR: Failed to parse the ONNX file.")
                    for error in range(parser.num_errors):
                        print(parser.get_error(error))
                    return None
                
            # The actual yolov3.onnx is generated with batch size 64. Reshape input to batch size 1
            # network.get_input(0).shape = [1, 3, 608, 608]
            print("Completed parsing of ONNX file")
            print("Building an engine from file {}; this may take a while...".format(onnx_file_path))
            plan = builder.build_serialized_network(network, config)
            engine = runtime.deserialize_cuda_engine(plan)
            print("Completed creating Engine")
            with open(engine_file_path, "wb") as f:
                f.write(plan)
            return engine

    return build_engine()
    # if os.path.exists(engine_file_path):
    #     # If a serialized engine exists, use it instead of building an engine.
    #     print("Reading engine from file {}".format(engine_file_path))
    #     with open(engine_file_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
    #         return runtime.deserialize_cuda_engine(f.read())
    # else:
    
import sys
import onnx
onnx_filename = EXP_NAME + ".onnx"
model = onnx.load(onnx_filename)
print(onnx.checker.check_model(model))

get_engine(onnx_filename, engine_file_path=EXP_NAME + ".trt")

