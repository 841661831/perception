//
// Created by root on 3/10/21.
//

#ifndef SECOND_DETECTOR_RPN_H
#define SECOND_DETECTOR_RPN_H
#include <iostream>
#include <vector>
#include <map>
#include <torch/script.h>
#include <torch/torch.h>



class RPNV2{
public:
    RPNV2(bool use_norm,
          int num_class,
          std::vector<int64_t> layer_nums,
          std::vector<int64_t> layer_strides,
          std::vector<int64_t> num_filters,
          std::vector<float> upsample_strides,
          std::vector<int64_t> num_upsample_filters,
          int64_t num_input_features,
          int64_t num_anchor_per_loc,
          bool encode_background_as_zeros,
          bool use_direction_classifier,
          bool use_groupnorm,
          int64_t num_groups,
          int64_t box_code_size,
          int64_t num_direction_bins,
          bool fuse_bn);
    RPNV2();
    torch::nn::Sequential make_layer(int64_t inplanes, int64_t planes, int64_t num_blocks, int stride = 1, bool fuse_bn = false);
    std::map<std::string, torch::Tensor> forward(torch::Tensor &x);
    void load_weight(std::map<std::string, torch::Tensor> &weight, bool eval, bool fuse_bn);
    void load_weight_fuse(std::map<std::string, torch::Tensor> &weight, bool eval);
    void fuse_conv_bn(std::map<std::string, torch::Tensor> &weight, bool eval);
private:
    int64_t _num_anchor_per_loc;
    int64_t _num_direction_bins;
    int64_t _num_class;
    bool _fuse_bn;
    bool _use_direction_classifier;
    int64_t _box_code_size;
    torch::nn::Conv2d _con_cls = torch::nn::Conv2d(torch::nn::Conv2dOptions(256, 32, 1).stride(1).padding(0).groups(1).bias(
            true));
    torch::nn::Conv2d _con_box = torch::nn::Conv2d(torch::nn::Conv2dOptions(256, 56, 1).stride(1).padding(0).groups(1).bias(
            true));
    torch::nn::Conv2d _conv_dir_cls = torch::nn::Conv2d(torch::nn::Conv2dOptions(256, 16, 1).stride(1).padding(0).groups(1).bias(
            true));

    std::vector<int64_t> _layer_strides;
    std::vector<int64_t> _num_filters;
    std::vector<int64_t> _layer_nums;
    std::vector<float> _upsample_strides;
    std::vector<int64_t> _num_upsample_filters;
    int64_t _num_input_features;
    bool _use_norm;
    bool _use_groupnorm;
    int64_t _num_groups;
    int64_t _upsample_start_idx;
    int64_t _num_out_filters;
    torch::nn::Sequential _block0;
    torch::nn::Sequential _deblock0;
    torch::nn::Sequential _block1;
    torch::nn::Sequential _deblock1;
    std::vector<torch::nn::Sequential> _module_list;

};


#endif //SECOND_DETECTOR_RPN_H
