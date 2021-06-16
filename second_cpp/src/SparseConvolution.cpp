//
// Created by root on 3/4/21.
//

#include "SparseConvolution.h"
#include <iostream>
#include <torch/script.h>
#include <vector>
#include <string>
#include <map>
#include <assert.h>
#include "ops.h"
#include "middle.h"




SparseConvolution::SparseConvolution(int ndim, int in_channels, int out_channels,
                                     std::vector<int64_t> kernel_size,
                                     std::vector<int64_t> stride,
                                     std::vector<int64_t> padding,
                                     std::vector<int64_t> dilation, int groups, bool subm,
                                     std::vector<int64_t> output_padding,
                                     bool transposed, bool inverse, std::string indice_key, bool fused_bn,
                                     bool use_hash, std::string algo) {
    assert(groups == 1);
    if (kernel_size.size() > 1){
        _kernel_size = kernel_size;
    } else{
        for (int i = 0; i < ndim; i++){
            _kernel_size.push_back(kernel_size[i]);
        }
    }
    if (stride.size() > 1){
        _stride = stride;
    } else{
        for (int i = 0; i < ndim; i++){
            _stride.push_back(stride[i]);
        }
    }
    if (padding.size() > 1){
        _padding = padding;
    } else{
        for (int i = 0; i < ndim; i++){
            _padding.push_back(padding[i]);
        }
    }
    _dilation = dilation;
    _output_padding = output_padding;
    _ndim = ndim;
    _in_channels = in_channels;
    _out_channels = out_channels;
    _conv1x1 = kernel_size[0] * kernel_size[1] * kernel_size[2] == 1;
    _groups = groups;
    _subm = subm;
    _transposed = transposed;
    _inverse = inverse;
    _indice_key = indice_key;
    _fused_bn = fused_bn;
    _use_hash = use_hash;
    if (algo == "Native")
        _algo = 0;
    if (algo == "Batch")
        _algo = 1;
    if (algo == "BatchGemmGather")
        _algo = 2;
//    torch::
//    _weight = _weight;    TODO by chenxiangyang done!
//    _bias;
}

SparseConvolution::SparseConvolution(){}
void SparseConvolution::load_weights(torch::Tensor &weights, torch::Tensor &bias) {
    _weight = weights;
    _bias = bias;
};

SparseConvTensor SparseConvolution::forward(SparseConvTensor &input) {
    torch::Tensor features = input.features;
//    std::cout<<features<<"\n";
    torch::Device device = features.device();
    torch::Tensor indices = input.indices;
//    std::cout<< indices << std::endl;
    std::vector<int64_t> spatial_shape = input.spatial_shape;
    int batch_size = input.batch_size;
    std::map<std::string, indice_struct> indice_dict = input.indice_dict;
    std::vector<int64_t> grid = input.grid;

    // get_indice_pairs results
    torch::Tensor outids_subm;
    torch::Tensor indices_subm;
    torch::Tensor indice_pairs_subm;
    torch::Tensor indice_pair_num_subm;
//    std::vector<int32_t> spatial_shape_subm;

    struct SparseConvTensor out_tensor; // output
    torch::Tensor out_features;
    std::vector<torch::Tensor> res;

    std::vector<int64_t>  out_spatial_shape;
    if(!_subm){
        if(_transposed){
//            TODO : ops.get_deconv_output_size,  not use
//            out_spatial_shape = ops.get_deconv_output_size(
//                    spatial_shape, self.kernel_size, self.stride, self.padding,
//                    self.dilation, self.output_padding)
        } else{
//              TODO : ops.get_conv_output_size
            out_spatial_shape = ops::get_conv_output_size(spatial_shape, _kernel_size, _stride, _padding, _dilation);
//            out_spatial_shape = ops.get_conv_output_size(
//                    spatial_shape, self.kernel_size, self.stride, self.padding,
//                    self.dilation)
        }
    } else
        out_spatial_shape = spatial_shape;

    if (_conv1x1){
        features = torch::mm(input.features,_weight.view({_in_channels, _out_channels})); // TODO : weights & bias done!
        if (_bias.sizes()[0] > 0){
            features += _bias;
        }
        out_tensor.features = features;
        out_tensor.indices = input.indices;
        out_tensor.spatial_shape = input.spatial_shape;
        out_tensor.batch_size = input.batch_size;
        out_tensor.indice_dict = input.indice_dict;
        out_tensor.grid = input.grid;
        return out_tensor;
    }

    indice_struct datas = input.find_indice_pair(_indice_key);
    if (_inverse){
        assert(datas.is_None == true and not _indice_key.empty());
        outids_subm = datas.outids;
        indice_pairs_subm = datas.indice_pairs;
        indice_pair_num_subm = datas.indice_pair_num;
        out_spatial_shape = datas.spatial_shape;
        assert(datas.indice_pair_num.sizes()[0] == _kernel_size[0] * _kernel_size[1] * _kernel_size[2]);
    } else{
        if (!_indice_key.empty() and !datas.is_None){
            outids_subm = datas.outids;
            indice_pairs_subm = datas.indice_pairs;
            indice_pair_num_subm = datas.indice_pair_num;
        } else{
//            TODO: by chenxiangyang done!
//            std::cout<< indices.sizes() << std::endl;
//            std::cout<< "spatial_shape " << spatial_shape << std::endl;
//            std::cout<< "_kernel_size " << _kernel_size << std::endl;
//            std::cout<< "_stride " << _stride << std::endl;
//            std::cout<< "_padding " << _padding << std::endl;
//            std::cout<< "_dilation " << _dilation << std::endl;
//            std::cout<< "_output_padding " << _output_padding << std::endl;
//            std::cout<< "_subm " << _subm << std::endl;
//            std::cout<< "_transposed " << _transposed << std::endl;
//            std::cout<< "_use_hash " << _use_hash << std::endl;
//            std::cout<< "grid " << grid << std::endl;
//            std::cout<< indices << std::endl;
            res = ops::get_indice_pairs(indices,
                                       batch_size,
                                       spatial_shape,
                                       _kernel_size,
                                       _stride,
                                       _padding,
                                       _dilation,
                                       _output_padding,
                                       _subm,
                                       _transposed,
                                       grid,
                                       _use_hash);
            outids_subm = res[0];
//            std::cout<<outids_subm << std::endl;
            indice_pairs_subm = res[1];
//            std::cout<<indice_pairs_subm << std::endl;
//            if (indice_pairs_subm.size(0) == 2 && indice_pairs_subm.size(1) == 27 && indice_pairs_subm.size(2) == 15249){
//                std::cout<< " ============== "<< indice_pair_num_subm[0][0][1] << std::endl;
//            }
            indice_pair_num_subm = res[2];
//            std::cout<<indice_pair_num_subm << std::endl;
            input.indice_dict[_indice_key].outids = outids_subm;
            input.indice_dict[_indice_key].indice_pairs = indice_pairs_subm;
            input.indice_dict[_indice_key].indice_pair_num = indice_pair_num_subm;
            input.indice_dict[_indice_key].indices = indices;
            input.indice_dict[_indice_key].spatial_shape = spatial_shape;
        }
    }
    if (_fused_bn){
        assert(_bias.sizes()[0] > 0);
//         TODO : by chenxiangyang. not use
//        out_features = ops.fused_indice_conv(features, self.weight,
//                                             self.bias,
//                                             indice_pairs.to(device),
//                                             indice_pair_num,
//                                             outids.shape[0], self.inverse,
//                                             self.subm)
    } else{
        if (_subm){
//            TODO : by chenxiangyang done!
//            std::cout<<features<<"\n";
//            std::cout<<_weight<<"\n";
            out_features = ops::indice_conv(features,
                                          _weight,
                                          indice_pairs_subm.to(device),
                                          indice_pair_num_subm,
                                          outids_subm.sizes()[0],
                                          _inverse,
                                          _subm,
                                          _algo);
//            std::cout<< "out_features: \n " << out_features << std::endl;

        } else{
            if (_inverse){
//                TODO : by chenxiangyang.
                out_features = ops::indice_conv(features,
                                                _weight,
                                                indice_pairs_subm.to(device),
                                                indice_pair_num_subm,
                                                outids_subm.sizes()[0],
                                                _inverse,
                                                _subm,
                                                _algo);
//                out_features = Fsp.indice_inverse_conv(
//                        features, self.weight, indice_pairs.to(device),
//                        indice_pair_num, outids.shape[0], self.algo)
            } else{
//                TODO : by chenxiangyang.
                out_features = ops::indice_conv(features,
                                                _weight,
                                                indice_pairs_subm.to(device),
                                                indice_pair_num_subm,
                                                outids_subm.sizes()[0],
                                                _inverse,
                                                _subm,
                                                _algo);
//                out_features = Fsp.indice_conv(features, self.weight,
//                                               indice_pairs.to(device),
//                                               indice_pair_num,
//                                               outids.shape[0], self.algo)
            }
        }
        if (_bias.sizes()[0] > 0){
            out_features += _bias;
        }
    }
//    std::cout<<out_features<<std::endl;
    out_tensor.features = out_features;
    out_tensor.indices = outids_subm;
    out_tensor.spatial_shape = out_spatial_shape;
    out_tensor.batch_size = batch_size;
    out_tensor.indice_dict = input.indice_dict;
    out_tensor.grid = input.grid;
//    delete input;
    return out_tensor;
};
