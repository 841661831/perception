//
// Created by root on 3/3/21.
//
#include <torch/script.h>
#include <torch/torch.h>
#include <vector>
#include <string>
#include <map>
#include "middle.h"
#include "SparseConvolution.h"

Mish::Mish() {}
torch::Tensor Mish::forward(torch::Tensor input) {
    return input * torch::tanh(torch::softplus(input));
}

BatchNorm::BatchNorm(torch::Tensor &weight, torch::Tensor &bias, torch::Tensor &mean, torch::Tensor &var, float momentum,
                     float eps, bool cuda_enabled):
                     _weight(weight),
                     _bias(bias),
                     _mean(mean),
                     _var(var),
                     _momentum(momentum),
                     _eps(eps),
                     _cuda_enabled(cuda_enabled)
                     {}
BatchNorm::BatchNorm() {}

torch::Tensor BatchNorm::forward(torch::Tensor &input) {
//    std::cout<<_var;
    return torch::batch_norm(input, _weight, _bias, _mean, _var, false, _momentum, _eps, _cuda_enabled);
}

SpMiddleFHD::SpMiddleFHD(std::vector<int64_t> &output_shape,
                         bool &use_norm,
                         int32_t &num_input_features,
                         std::map<std::string, torch::Tensor> &weights, bool kd):_weights(weights){
    _sparse_shape.resize(3);
    _sparse_shape[0] = output_shape[1] + 1;
    _sparse_shape[1] = output_shape[2];
    _sparse_shape[2] = output_shape[3];
    _voxel_output_shape = output_shape;
    _max_batch_size = 6;
    _use_norm = use_norm;
    _num_input_features = (int) num_input_features;

    _net_spconv = !kd ? net_conv() : net_conv_kd();
    _net_batchnorm = !kd ? net_bn() : net_bn_kd();
}
SpMiddleFHD::SpMiddleFHD() {}
std::map<int, SparseConvolution> SpMiddleFHD::net_conv() {
    std::map<int, SparseConvolution> net_conv;
//    std::cout<< _weights.size() <<std::endl;
    std::vector<int64_t> k1 = {3,3,3}, k2 = {3,1,1};
    std::vector<int64_t> s1 = {1,1,1}, s2 = {2,2,2}, s3 = {2,1,1};
    std::vector<int64_t> p1 = {0,0,0}, p2 = {1,1,1}, p3 = {0,1,1};
    std::vector<int64_t> dil = {1,1,1}, out_p = {0,0,0};
    net_conv[0] = SparseConvolution(3, _num_input_features, 16, k1, s1, p1, dil, 1, true, out_p, false, false, "subm0");
    net_conv[1] = SparseConvolution(3, 16, 16, k1, s1, p1, dil, 1, true, out_p, false, false, "subm0", false, false, "Batch");
    net_conv[2] = SparseConvolution(3, 16, 32, k1, s2, p2, dil, 1, false, out_p, false, false, "", false, false, "Batch");
    net_conv[3] = SparseConvolution(3, 32, 32, k1, s1, p1, dil, 1, true, out_p, false, false, "subm1", false, false, "Batch");
    net_conv[4] = SparseConvolution(3, 32, 32, k1, s1, p1, dil, 1, true, out_p, false, false, "subm1", false, false, "Batch");
    net_conv[5] = SparseConvolution(3, 32, 64, k1, s2, p2, dil, 1, false, out_p, false, false, "", false, false, "Batch");
    net_conv[6] = SparseConvolution(3, 64, 64, k1, s1, p1, dil, 1, true, out_p, false, false, "subm2", false, false, "Batch");
    net_conv[7] = SparseConvolution(3, 64, 64, k1, s1, p1, dil, 1, true, out_p, false, false, "subm2", false, false, "Batch");
    net_conv[8] = SparseConvolution(3, 64, 64, k1, s1, p1, dil, 1, true, out_p, false, false, "subm2", false, false, "Batch");
    net_conv[9] = SparseConvolution(3, 64, 64, k1, s2, p3, dil, 1, false, out_p, false, false, "", false, false, "Batch");
    net_conv[10] = SparseConvolution(3, 64, 64, k1, s1, p1, dil, 1, true, out_p, false, false, "subm3", false, false, "Batch");
    net_conv[11] = SparseConvolution(3, 64, 64, k1, s1, p1, dil, 1, true, out_p, false, false, "subm3", false, false, "Batch");
    net_conv[12] = SparseConvolution(3, 64, 64, k1, s1, p1, dil, 1, true, out_p, false, false, "subm3", false, false, "Batch");
    net_conv[13] = SparseConvolution(3, 64, 64, k2, s3, p1, dil, 1, false, out_p, false, false, "", false, false, "Batch");
//

    net_conv[0].load_weights(_weights["w0"], _weights["b0"]);
    net_conv[1].load_weights(_weights["w1"], _weights["b1"]);
    net_conv[2].load_weights(_weights["w2"], _weights["b2"]);
    net_conv[3].load_weights(_weights["w3"], _weights["b3"]);
    net_conv[4].load_weights(_weights["w4"], _weights["b4"]);
    net_conv[5].load_weights(_weights["w5"], _weights["b5"]);
    net_conv[6].load_weights(_weights["w6"], _weights["b6"]);
    net_conv[7].load_weights(_weights["w7"], _weights["b7"]);
    net_conv[8].load_weights(_weights["w8"], _weights["b8"]);
    net_conv[9].load_weights(_weights["w9"], _weights["b9"]);
    net_conv[10].load_weights(_weights["w10"], _weights["b10"]);
    net_conv[11].load_weights(_weights["w11"], _weights["b11"]);
    net_conv[12].load_weights(_weights["w12"], _weights["b12"]);
    net_conv[13].load_weights(_weights["w13"], _weights["b13"]);
    return net_conv;
}

std::map<int, torch::nn::Sequential> SpMiddleFHD::net_bn2(){
    std::map<int, torch::nn::Sequential> net_bn;
    std::vector<int> bn_channel = {16, 16, 32, 32,32, 64, 64, 64, 64, 64, 64, 64,64, 64};
    for (int i = 0; i < bn_channel.size(); ++i) {
        torch::nn::Sequential bn;
        bn->push_back(torch::nn::BatchNorm1d(torch::nn::BatchNorm1dOptions(bn_channel[i]).eps(0.001).momentum(0.01)));
        bn->push_back(torch::nn::ReLU());
        torch::nn::BatchNorm1dImpl *bn_imp = dynamic_cast<torch::nn::BatchNorm1dImpl *>(bn.ptr()->ptr(0).get());
        bn_imp->weight.set_data(_weights["bn_w" + std::to_string(i)].view_as(bn_imp->weight));
        bn_imp->bias.set_data(_weights["bn_b" + std::to_string(i)].view_as(bn_imp->bias));
        bn_imp->running_mean.set_data(_weights["bn_m" + std::to_string(i)].view_as(bn_imp->running_mean));
        bn_imp->running_var.set_data(_weights["bn_v" + std::to_string(i)].view_as(bn_imp->running_var));
        bn->eval();
        net_bn[i] = bn;
    }

    return net_bn;
}


std::map<int, BatchNorm> SpMiddleFHD::net_bn() {
    std::map<int, BatchNorm> net_bn;

//    auto a = BatchNorm(_weights["bn_w0"], _weights["bn_b0"], _weights["bn_m0"], _weights["bn_v0"]);
    net_bn[0] = BatchNorm(_weights["bn_w0"], _weights["bn_b0"], _weights["bn_m0"], _weights["bn_v0"]);
//    std::cout<<net_bn[0]._weight;
    net_bn[1] = BatchNorm(_weights["bn_w1"], _weights["bn_b1"], _weights["bn_m1"], _weights["bn_v1"]);
    net_bn[2] = BatchNorm(_weights["bn_w2"], _weights["bn_b2"], _weights["bn_m2"], _weights["bn_v2"]);
    net_bn[3] = BatchNorm(_weights["bn_w3"], _weights["bn_b3"], _weights["bn_m3"], _weights["bn_v3"]);
    net_bn[4] = BatchNorm(_weights["bn_w4"], _weights["bn_b4"], _weights["bn_m4"], _weights["bn_v4"]);
    net_bn[5] = BatchNorm(_weights["bn_w5"], _weights["bn_b5"], _weights["bn_m5"], _weights["bn_v5"]);
    net_bn[6] = BatchNorm(_weights["bn_w6"], _weights["bn_b6"], _weights["bn_m6"], _weights["bn_v6"]);
    net_bn[7] = BatchNorm(_weights["bn_w7"], _weights["bn_b7"], _weights["bn_m7"], _weights["bn_v7"]);
    net_bn[8] = BatchNorm(_weights["bn_w8"], _weights["bn_b8"], _weights["bn_m8"], _weights["bn_v8"]);
    net_bn[9] = BatchNorm(_weights["bn_w9"], _weights["bn_b9"], _weights["bn_m9"], _weights["bn_v9"]);
    net_bn[10] = BatchNorm(_weights["bn_w10"], _weights["bn_b10"], _weights["bn_m10"], _weights["bn_v10"]);
    net_bn[11] = BatchNorm(_weights["bn_w11"], _weights["bn_b11"], _weights["bn_m11"], _weights["bn_v11"]);
    net_bn[12] = BatchNorm(_weights["bn_w12"], _weights["bn_b12"], _weights["bn_m12"], _weights["bn_v12"]);
    net_bn[13] = BatchNorm(_weights["bn_w13"], _weights["bn_b13"], _weights["bn_m13"], _weights["bn_v13"]);
    return net_bn;
}
torch::Tensor SpMiddleFHD::forward(torch::Tensor &voxel_features, torch::Tensor &coors, int &batch_size) {
    auto t1 = std::chrono::steady_clock::now();
//    std::cout<<voxel_features<<"\n";
    coors = coors.to(torch::kInt32);
    struct SparseConvTensor ret;

    ret.features = voxel_features;
    ret.indices = coors;
    ret.spatial_shape = _sparse_shape;
    ret.batch_size = batch_size;
    torch::nn::ReLU relu = torch::nn::ReLU();
    Mish mish = Mish(); // 32 line
    auto t2 = std::chrono::steady_clock::now();
//    int layers = _net_spconv.size();
    for(int i = 0; i < _net_spconv.size(); i++){
        ret = _net_spconv[i].forward(ret);
        ret.features = _net_batchnorm[i].forward(ret.features);
        ret.features = relu->forward(ret.features);
//        ret.features = mish.forward(ret.features);
    }
    auto t3 = std::chrono::steady_clock::now();
    torch::Tensor res = ret.features;
    ops::dense(res, ret.spatial_shape, ret.indices, ret.batch_size);
    auto t4 = std::chrono::steady_clock::now();

    auto N = res.size(0);
    auto C = res.size(1);
    auto D = res.size(2);
    auto H = res.size(3);
    auto W = res.size(4);
    res = res.view({N, C * D, H, W});
    auto t5 = std::chrono::steady_clock::now();

    double middle_trans = std::chrono::duration<double, std::milli>(t2 - t1).count();
    double middle_forward = std::chrono::duration<double, std::milli>(t3 - t2).count();
    double middle_dense = std::chrono::duration<double, std::milli>(t4 - t3).count();
    double middle_resize = std::chrono::duration<double, std::milli>(t5 - t4).count();
//    printf("middle_trans cost %4f ms!\n", middle_trans);
    printf("middle_forward cost %4f ms!\n", middle_forward);
    printf("middle_dense cost %4f ms!\n", middle_dense);
//    printf("middle_resize cost %4f ms!\n", middle_resize);

    return res;
}

std::map<int, SparseConvolution> SpMiddleFHD::net_conv_kd() {
    std::map<int, SparseConvolution> net_conv;
//    std::cout<< _weights.size() <<std::endl;
    std::vector<int64_t> k1 = {3,3,3}, k2 = {3,1,1};
    std::vector<int64_t> s1 = {1,1,1}, s2 = {2,2,2}, s3 = {2,1,1};
    std::vector<int64_t> p1 = {0,0,0}, p2 = {1,1,1}, p3 = {0,1,1};
    std::vector<int64_t> dil = {1,1,1}, out_p = {0,0,0};
    net_conv[0] = SparseConvolution(3, _num_input_features, 16, k1, s1, p1, dil, 1, true, out_p, false, false, "subm0");

    net_conv[1] = SparseConvolution(3, 16, 32, k1, s2, p2, dil, 1, false, out_p, false, false, "", false, false, "Batch");
    net_conv[2] = SparseConvolution(3, 32, 32, k1, s1, p1, dil, 1, true, out_p, false, false, "subm1", false, false, "Batch");

    net_conv[3] = SparseConvolution(3, 32, 64, k1, s2, p2, dil, 1, false, out_p, false, false, "", false, false, "Batch");
    net_conv[4] = SparseConvolution(3, 64, 64, k1, s1, p1, dil, 1, true, out_p, false, false, "subm2", false, false, "Batch");

    net_conv[5] = SparseConvolution(3, 64, 64, k1, s2, p3, dil, 1, false, out_p, false, false, "", false, false, "Batch");
    net_conv[6] = SparseConvolution(3, 64, 64, k1, s1, p1, dil, 1, true, out_p, false, false, "subm3", false, false, "Batch");

    net_conv[7] = SparseConvolution(3, 64, 64, k2, s3, p1, dil, 1, false, out_p, false, false, "", false, false, "Batch");


    net_conv[0].load_weights(_weights["w0"], _weights["b0"]);
    net_conv[1].load_weights(_weights["w1"], _weights["b1"]);
    net_conv[2].load_weights(_weights["w2"], _weights["b2"]);
    net_conv[3].load_weights(_weights["w3"], _weights["b3"]);
    net_conv[4].load_weights(_weights["w4"], _weights["b4"]);
    net_conv[5].load_weights(_weights["w5"], _weights["b5"]);
    net_conv[6].load_weights(_weights["w6"], _weights["b6"]);
    net_conv[7].load_weights(_weights["w7"], _weights["b7"]);

    return net_conv;
}

std::map<int, BatchNorm> SpMiddleFHD::net_bn_kd(){
    std::map<int, BatchNorm> net_bn;

//    auto a = BatchNorm(_weights["bn_w0"], _weights["bn_b0"], _weights["bn_m0"], _weights["bn_v0"]);
    net_bn[0] = BatchNorm(_weights["bn_w0"], _weights["bn_b0"], _weights["bn_m0"], _weights["bn_v0"]);
//    std::cout<<net_bn[0]._weight;
    net_bn[1] = BatchNorm(_weights["bn_w1"], _weights["bn_b1"], _weights["bn_m1"], _weights["bn_v1"]);
    net_bn[2] = BatchNorm(_weights["bn_w2"], _weights["bn_b2"], _weights["bn_m2"], _weights["bn_v2"]);
    net_bn[3] = BatchNorm(_weights["bn_w3"], _weights["bn_b3"], _weights["bn_m3"], _weights["bn_v3"]);
    net_bn[4] = BatchNorm(_weights["bn_w4"], _weights["bn_b4"], _weights["bn_m4"], _weights["bn_v4"]);
    net_bn[5] = BatchNorm(_weights["bn_w5"], _weights["bn_b5"], _weights["bn_m5"], _weights["bn_v5"]);
    net_bn[6] = BatchNorm(_weights["bn_w6"], _weights["bn_b6"], _weights["bn_m6"], _weights["bn_v6"]);
    net_bn[7] = BatchNorm(_weights["bn_w7"], _weights["bn_b7"], _weights["bn_m7"], _weights["bn_v7"]);
//    net_bn[8] = BatchNorm(_weights["bn_w8"], _weights["bn_b8"], _weights["bn_m8"], _weights["bn_v8"]);
//    net_bn[9] = BatchNorm(_weights["bn_w9"], _weights["bn_b9"], _weights["bn_m9"], _weights["bn_v9"]);
//    net_bn[10] = BatchNorm(_weights["bn_w10"], _weights["bn_b10"], _weights["bn_m10"], _weights["bn_v10"]);
//    net_bn[11] = BatchNorm(_weights["bn_w11"], _weights["bn_b11"], _weights["bn_m11"], _weights["bn_v11"]);
//    net_bn[12] = BatchNorm(_weights["bn_w12"], _weights["bn_b12"], _weights["bn_m12"], _weights["bn_v12"]);
//    net_bn[13] = BatchNorm(_weights["bn_w13"], _weights["bn_b13"], _weights["bn_m13"], _weights["bn_v13"]);
    return net_bn;
}
