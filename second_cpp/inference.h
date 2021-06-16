//
// Created by root on 3/23/21.
//

#ifndef SECOND_DETECTOR_INFERENCE_H
#define SECOND_DETECTOR_INFERENCE_H
#include <iostream>
#include <map>
#include <torch/script.h>
#include <torch/torch.h>
#include <ATen/ATen.h>
#include <vector>
#include <string>
#include "src/SparseConvolution.h"
#include "src/middle.h"
#include "src/rpn.h"
#include "src/cnpy.h"
#include "VoxelNet.h"
#include "src/voxel_encoder.h"



torch::Tensor npy2tensor(std::string npy_path, torch::Device device = torch::kCUDA, bool is_int = false);
torch::Tensor test_middle();
void test_voxel_generate();
void test_voxel_generate_vfe();
void test_fuse();
void test_rpn();
torch::Tensor test_spconv();
VoxelGeneratorV2 construct_voxel_generator();
VoxelNet construct_voxelnet(bool kd = false);

#endif //SECOND_DETECTOR_INFERENCE_H
