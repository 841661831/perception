//
// Created by root on 3/4/21.
//

#ifndef SPARSECONVOLUTIONPLUGIN_SPARSECONVOLUTION_H
#define SPARSECONVOLUTIONPLUGIN_SPARSECONVOLUTION_H

#include <iostream>
#include <torch/script.h>
#include <vector>
#include <string>
#include <map>
#include "ops.h"
//#include "VoxelNet.h"


struct indice_struct{
    torch::Tensor outids;
    torch::Tensor indices;
    torch::Tensor indice_pairs;
    torch::Tensor indice_pair_num;
    std::vector<int64_t> spatial_shape;
    bool is_None;
};

struct SparseConvTensor{
    torch::Tensor features;
    torch::Tensor indices;
    std::vector<int64_t> spatial_shape;
    int batch_size;
    std::map<std::string, indice_struct> indice_dict;
    std::vector<int64_t> grid;

    indice_struct find_indice_pair(std::string &indice_key){
        struct indice_struct a;
        a.is_None = true;
        if (indice_key.empty())
            return a;
        if(indice_dict.find(indice_key) != indice_dict.end()){
            return indice_dict[indice_key];
        } else
            return a;
    };

};

class SparseConvolution{
public:
    SparseConvolution(int ndim, int in_channels, int out_channels,
                      std::vector<int64_t> kernel_size,
                      std::vector<int64_t> stride,
                      std::vector<int64_t> padding,
                      std::vector<int64_t>  dilation,
                      int groups, bool subm,
                      std::vector<int64_t> output_padding,
                      bool transposed, bool inverse,
                      std::string indice_key, bool fused_bn = false, bool use_hash = false,
                      std::string algo = "Batch");

    SparseConvolution();
    SparseConvTensor forward(SparseConvTensor &input);
    void load_weights(torch::Tensor &weights, torch::Tensor &bias);
private:
    int _ndim;
    int _in_channels;
    int _out_channels;
    std::vector<int64_t> _kernel_size;
    bool _conv1x1;
    std::vector<int64_t>  _stride;
    std::vector<int64_t>  _padding;
    std::vector<int64_t>  _dilation;
    int _groups;
    bool _subm;
    std::vector<int64_t>  _output_padding;
    bool _transposed;
    bool _inverse;
    std::string _indice_key;
    bool _fused_bn;
    bool _use_hash;
    int _algo;
    torch::Tensor _weight;
    torch::Tensor _bias;
//    self.register_parameter
};


#endif //SPARSECONVOLUTIONPLUGIN_SPARSECONVOLUTION_H