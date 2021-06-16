//
// Created by root on 3/3/21.
//

#ifndef SPARSECONVOLUTIONPLUGIN_VOXELNET_H
#define SPARSECONVOLUTIONPLUGIN_VOXELNET_H

#include <iostream>
#include <torch/script.h>
#include <vector>
#include <string>
#include <map>
#include "SparseConvolution.h"

class Mish{
public:
    Mish();
    torch::Tensor forward(torch::Tensor input);
};

class BatchNorm{
public:
    BatchNorm(torch::Tensor &weight, torch::Tensor &bias, torch::Tensor &mean, torch::Tensor &var, float momentum = 0.01, float eps = 0.001, bool cuda_enabled = true);
    BatchNorm();
    torch::Tensor forward(torch::Tensor &input);
private:
    torch::Tensor _weight;
    torch::Tensor _bias;
    torch::Tensor _mean;
    torch::Tensor _var;
    float _momentum;
    float _eps;
    bool _cuda_enabled;
};

class SpMiddleFHD{
public:
    SpMiddleFHD(std::vector<int64_t> &output_shape, bool &use_norm, int32_t &num_input_features,
                std::map<std::string, torch::Tensor> &weights, bool kd = false);
    SpMiddleFHD();
    torch::Tensor forward(torch::Tensor &voxel_features, torch::Tensor &coors, int &batch_size);
    std::map<int, SparseConvolution> net_conv();
    std::map<int, BatchNorm> net_bn();
    std::map<int, torch::nn::Sequential> net_bn2();

    // KD
    std::map<int, SparseConvolution> net_conv_kd();
    std::map<int, BatchNorm> net_bn_kd();
//    SparseConvTensor
//    std::vector<SparseConvolution> middle_conv();
private:

    std::vector<int64_t> _sparse_shape;
    std::vector<int64_t> _voxel_output_shape;
//    bool middle_conv; // spconv
    int _max_batch_size;
    bool _use_norm;
    int _num_input_features;
    std::map<std::string, torch::Tensor> _weights;

    std::map<int, SparseConvolution> _net_spconv ;
//    std::map<int, torch::nn::Sequential> _net_batchnorm;
    std::map<int, BatchNorm> _net_batchnorm;

};


#endif //SPARSECONVOLUTIONPLUGIN_VOXELNET_H
