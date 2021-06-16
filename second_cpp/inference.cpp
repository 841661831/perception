//
// Created by root on 3/23/21.
//
#include <iostream>
#include <map>
#include <torch/script.h>
#include <torch/torch.h>
#include <ATen/ATen.h>
#include <vector>
#include <string>
#include "src/target_assigner.h"
#include "src/SparseConvolution.h"
#include "src/middle.h"
#include "src/rpn.h"
#include "src/cnpy.h"
#include "VoxelNet.h"
#include "src/voxel_encoder.h"
#include <c10/util/ArrayRef.h>
#include <stdlib.h>
#include <unistd.h>
#include <xtensor/xarray.hpp>
#include "inference.h"


#define voxels  {0.05, 0.05, 0.1}
#define point_cloud_ranges {0, -32.0, -3, 52.8, 32.0, 1}
//#define voxels  {0.1, 0.1, 0.1}
//#define point_cloud_ranges {-70.4, -70.4, -5.3, 70.4, 70.4, -0.3}

torch::Tensor npy2tensor(std::string npy_path, torch::Device device, bool is_int){
    cnpy::NpyArray arr = cnpy::npy_load(npy_path);

    std::vector<size_t> arr_shape = arr.shape;
//    std::cout<< arr_shape.size() << std::endl;
    if (arr_shape.size() > 0){
        std::vector<int64_t> vec_arr(arr_shape.size());
        for (int idx = 0; idx < arr_shape.size(); idx++){
            vec_arr[idx] = arr_shape.at(idx);
        }
        c10::ArrayRef<int64_t> shape = c10::ArrayRef<int64_t>(vec_arr);
        if(is_int) {
            std::vector<int> arr_ = arr.as_vec<int>();
            torch::Tensor weight = torch::tensor(arr_, torch::TensorOptions().dtype(torch::kInt32).device(device)); // vector 2 tensor
            weight = weight.reshape(shape);
//            std::cout<< npy_path << weight.sizes()<<std::endl;
//            std::cout<<weight<<std::endl;
            return weight;

        } else{
            std::vector<float_t> arr_ = arr.as_vec<float_t>();
            torch::Tensor weight = torch::tensor(arr_, torch::TensorOptions().dtype(torch::kFloat32).device(device)); // vector 2 tensor
            weight = weight.reshape(shape);
//            std::cout<< npy_path << weight.sizes()<<std::endl;
//            std::cout<<weight<<std::endl;
            return weight;
        }

    } else{
        return torch::zeros({0});
    }

}
//void test_load_weight(){
//
//    ifstream fs("/data/second_cpp/weights/voxelnet-619.tckpt", ios::binary);
//
//    if (!fs) {
//        std::cout << "Fail to load weight file: " << "/data/second_cpp/yolov3.weights" << endl;
//        std::cout <<  strerror(errno) << endl;
//        exit(-1);
//    }
//    // header info: 5 * int32_t
//    int32_t header_size = sizeof(int32_t)*5;
//
//    int64_t index_weight = 0;
//
//    fs.seekg (0, fs.end);
//    int64_t length = fs.tellg();
//    // skip header
//    length = length - header_size;
//
//    fs.seekg (header_size, fs.beg);
//    float *weights_src = (float *)malloc(length);
//    fs.read(reinterpret_cast<char*>(weights_src), length);
//
//    fs.close();
//
//    /*at::TensorOptions options= torch::TensorOptions()
//        .dtype(torch::kFloat32)
//        .is_variable(true);*/ //@ihmc3jn09hk Remove unused code
//    at::Tensor weights = torch::from_blob(weights_src, {length/4});
//    std::cout<<weights.sizes()<<std::endl;
//}
torch::Tensor test_middle(){
    torch::Tensor bias = torch::zeros({0}).to(torch::kCUDA);
    std::map<std::string, torch::Tensor> weights;
    for(int i = 0; i < 14; i++){
        weights["w" + std::to_string(i)] = npy2tensor("/data/second_cpp/weights/middle2/conv" + std::to_string(i) + ".npy");
        weights["b" + std::to_string(i)] = bias;
        weights["bn_w" + std::to_string(i)] = npy2tensor("/data/second_cpp/weights/middle2/bn" + std::to_string(i) + "_w.npy");
        weights["bn_b" + std::to_string(i)] = npy2tensor("/data/second_cpp/weights/middle2/bn" + std::to_string(i) + "_b.npy");
        weights["bn_m" + std::to_string(i)] = npy2tensor("/data/second_cpp/weights/middle2/bn" + std::to_string(i) + "_mean.npy");
        weights["bn_v" + std::to_string(i)] = npy2tensor("/data/second_cpp/weights/middle2/bn" + std::to_string(i) + "_var.npy");
    }
//    std::cout<<weights["b0"] << std::endl;
    torch::Tensor voxel_features = npy2tensor("/data/second_cpp/weights/middle2/voxel_features.npy");
    torch::Tensor coors = npy2tensor("/data/second_cpp/weights/middle2/coors.npy", torch::kCUDA, true);

//    SpMiddleFHD *middle2 = SpMiddleFHD();
    std::vector<int64_t> output_shape = {1, 40, 1280, 1056, 16};
    bool use_norm = true;
    int32_t middle_num_input_features = 3;
    int batch_size = 1;
    SpMiddleFHD middle = SpMiddleFHD(output_shape, use_norm, middle_num_input_features, weights);
//    clock_t start = clock();
    auto res = middle.forward(voxel_features, coors, batch_size);
//    clock_t end = clock();
//    double t = (double) (end - start) / CLOCKS_PER_SEC; // s
//    std::cout<< "cost time : "<< t * 1000 << " ms!"<<std::endl;

    return res;
}
//int test_voxelnet(){
////    test_voxel_generate_vfe();
//    std::map<std::string, torch::Tensor> weight_rpn;
//    for (int i = 0; i < 2; i++){
//        for (int j = 0; j < 18; ++j) {
//
//            if (j % 3 == 0) {
//                if (j == 0) {
//                    std::string dconv_w = "db_" + std::to_string(i) + "_" + std::to_string(j) + "_w";
//                    std::string dconv_w_f =
//                            "/data/second_cpp/weights/rpn/deblocks_" + std::to_string(i) + "_" + std::to_string(j) +
//                            "_weight.npy";
////                    std::cout<<dconv_w_f << std::endl;
//                    weight_rpn[dconv_w] = npy2tensor(dconv_w_f);
//                }
//            }
//            if (j % 3 == 1){
//                std::string conv_w = "b_" + std::to_string(i) + "_" + std::to_string(j) + "_w";
//                std::string conv_w_f = "/data/second_cpp/weights/rpn/blocks_" + std::to_string(i) + "_" + std::to_string(j) + "_weight.npy";
//                weight_rpn[conv_w] = npy2tensor(conv_w_f);
//                if (j == 1){
//                    std::string dbn_w = "dbn_" + std::to_string(i) + "_" + std::to_string(j) + "_w";
//                    std::string dbn_w_f = "/data/second_cpp/weights/rpn/deblocks_" + std::to_string(i) + "_" + std::to_string(j) + "_weight.npy";
//                    std::string dbn_b = "dbn_" + std::to_string(i) + "_" + std::to_string(j) + "_b";
//                    std::string dbn_b_f = "/data/second_cpp/weights/rpn/deblocks_" + std::to_string(i) + "_" + std::to_string(j) + "_bias.npy";
//                    std::string dbn_m = "dbn_" + std::to_string(i) + "_" + std::to_string(j) + "_m";
//                    std::string dbn_m_f = "/data/second_cpp/weights/rpn/deblocks_" + std::to_string(i) + "_" + std::to_string(j) + "_running_mean.npy";
//                    std::string dbn_v = "dbn_" + std::to_string(i) + "_" + std::to_string(j) + "_v";
//                    std::string dbn_v_f = "/data/second_cpp/weights/rpn/deblocks_" + std::to_string(i) + "_" + std::to_string(j) + "_running_var.npy";
//                    std::string dbn_t = "dbn_" + std::to_string(i) + "_" + std::to_string(j) + "_t";
//                    std::string dbn_t_f = "/data/second_cpp/weights/rpn/deblocks_" + std::to_string(i) + "_" + std::to_string(j) + "_num_batches_tracked.npy";
//                    weight_rpn[dbn_w] = npy2tensor(dbn_w_f);
//                    weight_rpn[dbn_b] = npy2tensor(dbn_b_f);
//                    weight_rpn[dbn_m] = npy2tensor(dbn_m_f);
//                    weight_rpn[dbn_v] = npy2tensor(dbn_v_f);
//                    weight_rpn[dbn_t] = npy2tensor(dbn_t_f);
//                }
//            }
//            if (j % 3 == 2){
//                std::string bn_w = "bn_" + std::to_string(i) + "_" + std::to_string(j) + "_w";
//                std::string bn_w_f = "/data/second_cpp/weights/rpn/blocks_" + std::to_string(i) + "_" + std::to_string(j) + "_weight.npy";
//                std::string bn_b = "bn_" + std::to_string(i) + "_" + std::to_string(j) + "_b";
//                std::string bn_b_f = "/data/second_cpp/weights/rpn/blocks_" + std::to_string(i) + "_" + std::to_string(j) + "_bias.npy";
//                std::string bn_m = "bn_" + std::to_string(i) + "_" + std::to_string(j) + "_m";
//                std::string bn_m_f = "/data/second_cpp/weights/rpn/blocks_" + std::to_string(i) + "_" + std::to_string(j) + "_running_mean.npy";
//                std::string bn_v = "bn_" + std::to_string(i) + "_" + std::to_string(j) + "_v";
//                std::string bn_v_f = "/data/second_cpp/weights/rpn/blocks_" + std::to_string(i) + "_" + std::to_string(j) + "_running_var.npy";
//                std::string bn_t = "bn_" + std::to_string(i) + "_" + std::to_string(j) + "_t";
//                std::string bn_t_f = "/data/second_cpp/weights/rpn/blocks_" + std::to_string(i) + "_" + std::to_string(j) + "_num_batches_tracked.npy";
//
//                weight_rpn[bn_w] = npy2tensor(bn_w_f);
//                weight_rpn[bn_b] = npy2tensor(bn_b_f);
//                weight_rpn[bn_m] = npy2tensor(bn_m_f);
//                weight_rpn[bn_v] = npy2tensor(bn_v_f);
//                weight_rpn[bn_t] = npy2tensor(bn_t_f);
//            }
//        }
//    }
//    weight_rpn["cls_w"] = npy2tensor("/data/second_cpp/weights/rpn/conv_cls_weight.npy");
//    weight_rpn["cls_b"] = npy2tensor("/data/second_cpp/weights/rpn/conv_cls_bias.npy");
//    weight_rpn["box_w"] = npy2tensor("/data/second_cpp/weights/rpn/conv_box_weight.npy");
//    weight_rpn["box_b"] = npy2tensor("/data/second_cpp/weights/rpn/conv_box_bias.npy");
//    weight_rpn["dir_w"] = npy2tensor("/data/second_cpp/weights/rpn/conv_dir_cls_weight.npy");
//    weight_rpn["dir_b"] = npy2tensor("/data/second_cpp/weights/rpn/conv_dir_cls_bias.npy");
//    std::map<std::string, torch::Tensor> weights_mid;
//    torch::Tensor bias = torch::zeros({0}).to(torch::kCUDA);
//    for(int i = 0; i < 14; i++){
//        weights_mid["w" + std::to_string(i)] = npy2tensor("/data/second_cpp/weights/middle2/conv" + std::to_string(i) + ".npy");
//        weights_mid["b" + std::to_string(i)] = bias;
//        weights_mid["bn_w" + std::to_string(i)] = npy2tensor("/data/second_cpp/weights/middle2/bn" + std::to_string(i) + "_w.npy");
//        weights_mid["bn_b" + std::to_string(i)] = npy2tensor("/data/second_cpp/weights/middle2/bn" + std::to_string(i) + "_b.npy");
//        weights_mid["bn_m" + std::to_string(i)] = npy2tensor("/data/second_cpp/weights/middle2/bn" + std::to_string(i) + "_mean.npy");
//        weights_mid["bn_v" + std::to_string(i)] = npy2tensor("/data/second_cpp/weights/middle2/bn" + std::to_string(i) + "_var.npy");
//    }
//    std::vector<int64_t> output_shape = {1, 40, 1280, 1056, 16};
//    std::vector<float> voxel_size = {0.05, 0.05, 0.1};
//    std::vector<float> point_cloud_range = {0, -32.0, -3, 52.8, 32.0, 1};
//    int64_t max_num_points = 5;
//    int64_t max_voxels = 20000;
//    bool full_mean = false;
//    bool block_filtering = false;
//    int64_t block_factor = 0;
//    int64_t block_size = 0;
//    float height_threshold = 0.0;
//    float height_high_threshold = 2.0;
//    int num_class = 4;
//    int64_t num_input_features = 4;
//    std::vector<int> vfe_num_filters = {16};
//    bool with_distance = false;
//    int32_t middle_num_input_features = 3;
//    int64_t rpn_num_input_features = 128;
//    int64_t num_anchor_per_loc = 8;
//    std::vector<int64_t> rpn_layer_nums = {5, 5};
//    std::vector<int64_t> rpn_layer_strides = {1,2};
//    std::vector<int64_t> rpn_num_filters = {64,128};
//    std::vector<float> rpn_upsample_strides = {1.0, 2.0};
//    std::vector<int64_t> rpn_num_upsample_filters = {128, 128};
//    bool use_norm = true;
//    bool use_groupnorm = false;
//    int64_t num_groups = 32;
//    int64_t box_code_size = 7;
//    bool use_direction_classifier = true;
//    bool encode_background_as_zeros = true;
//    int64_t num_direction_bins = 2;
//    std::map<std::string, std::map<std::string, torch::Tensor>> weights;
//    weights["middle2"] = weights_mid;
//    weights["rpn"] = weight_rpn;
//    std::map<std::string, torch::Tensor> example;
//    example["voxels"] = npy2tensor("/data/second_cpp/weights/voxel/voxels.npy");
//    example["coordinates"] = npy2tensor("/data/second_cpp/weights/voxel/coordinates.npy", torch::kCUDA, true);
//    example["num_points"] = npy2tensor("/data/second_cpp/weights/voxel/num_points.npy", torch::kCUDA, true);
////    example["anchors"] = npy2tensor("/data/second_cpp/weights/voxel/anchors.npy");
////    std::cout<<example["num_points"]<<std::endl;
//    VoxelNet voxelNet = VoxelNet(output_shape, voxel_size,  point_cloud_range, max_num_points,  max_voxels,
//                                 full_mean,  block_filtering,  block_factor, block_size, height_threshold,
//                                 height_high_threshold,  weights, false,
//                                 num_class,  num_input_features,  vfe_num_filters, with_distance,  middle_num_input_features,
//                                 rpn_num_input_features, num_anchor_per_loc, rpn_layer_nums, rpn_layer_strides,  rpn_num_filters,
//                                 rpn_upsample_strides,  rpn_num_upsample_filters,  use_norm,  use_groupnorm,  num_groups,  box_code_size,
//                                 use_direction_classifier, encode_background_as_zeros,  num_direction_bins);
//    for (int i = 0; i < 100; ++i) {
//        auto res = voxelNet.forward(example, 1);
//    }
//    return 0;
//}
// TODO :
//      1: voxel_generate is too slow!  use xtensor instead! 2021.03.19
//      2: done! 2020.03.23
//void test_voxel_generate(){
//    /**
//    torch::Tensor points = npy2tensor("/data/second_cpp/weights/vfe/000001.npy", torch::kCUDA);
//    std::vector<float> voxel_size = {0.05, 0.05, 0.1};
//    std::vector<float> point_cloud_range = {0, -32.0, -3, 52.8, 32.0, 1};
//    int64_t max_num_points = 5;
//    int64_t max_voxels = 20000;
//    **/
//    std::string path = "/data/second_cpp/weights/vfe/000001.npy";
//    xt::xarray<float> points= xt::load_npy<float>(path);
//    points.reshape({-1, 4});
////    std::cout<<points.shape(0) << std::endl;
//    std::vector<float> voxel_size = {0.05, 0.05, 0.1};
//    std::vector<float> point_cloud_range = {0, -32.0, -3, 52.8, 32.0, 1};
//    int64_t max_num_points = 5;
//    int64_t max_voxels = 20000;
//    VoxelGeneratorV2  voxel_generate = VoxelGeneratorV2(voxel_size, point_cloud_range, max_num_points, max_voxels, false,
//                                                        false, 0, 0, 0.0, 2.0);
//    clock_t stat = clock();
//    // voxel generate
//    auto res = voxel_generate.points_to_voxel(points, 90000);
//    // test time
//    clock_t end = clock();
//    double cost_time = (double) (end - stat) / CLOCKS_PER_SEC;
//    std::cout<< "voxel generate : " <<cost_time * 1000 << std::endl;
//}
//void test_voxel_generate_vfe(){
//    std::string path = "/data/second_cpp/weights/vfe/000001.npy";
//    xt::xarray<float> points= xt::load_npy<float>(path);
//    points.reshape({-1, 4});
//    std::cout<<points.shape(0) << std::endl;
//    std::vector<float> voxel_size = {0.05, 0.05, 0.1};
//    std::vector<float> point_cloud_range = {0, -32.0, -3, 52.8, 32.0, 1};
//    int64_t max_num_points = 5;
//    int64_t max_voxels = 20000;
//    VoxelGeneratorV2  voxel_generate = VoxelGeneratorV2(voxel_size, point_cloud_range, max_num_points, max_voxels, false,
//                                                        false, 0, 0, 0.0, 2.0);
//    auto res = voxel_generate.points_to_voxel(points, 90000);
//
//
////    ['voxels', 'coordinates', 'num_points_per_voxel', 'voxel_point_mask', 'voxel_num']
////    auto res_voxels = npy2tensor("/data/second_cpp/weights/vfe/res_voxels.npy", torch::kCPU);
////    auto res_coordinates = npy2tensor("/data/second_cpp/weights/vfe/res_coordinates.npy", torch::kCPU, true);
////    auto res_num_points_per_voxel = npy2tensor("/data/second_cpp/weights/vfe/res_num_points_per_voxel.npy", torch::kCPU,
////                                               true);
////    auto res_voxel_point_mask = npy2tensor("/data/second_cpp/weights/vfe/res_voxel_point_mask.npy", torch::kCPU);
////    auto res_voxel_num = npy2tensor("/data/second_cpp/weights/vfe/res_voxel_num.npy", torch::kCPU, true);
//
////    xt::xarray<float> res_voxels= xt::load_npy<float>("/data/second_cpp/weights/vfe/res_voxels.npy");
////    xt::xarray<int> res_coordinates = xt::load_npy<int32_t>("/data/second_cpp/weights/vfe/res_coordinates.npy");
////    xt::xarray<int> res_num_points_per_voxel = xt::load_npy<int>("/data/second_cpp/weights/vfe/res_num_points_per_voxel.npy");
////    xt::xarray<float> res_voxel_point_mask = xt::load_npy<float>("/data/second_cpp/weights/vfe/res_voxel_point_mask.npy");
////    std::cout<<"res_voxel_point_mask : " << res_voxel_point_mask.shape(2) << std::endl;
////
////    auto res_voxels_t = torch::from_blob(res_voxels.data(), {43950, 5, 4}).to(torch::kFloat32);
////    auto res_coordinates_t = torch::from_blob(res_coordinates.data(), {43950, 3}).to(torch::kInt32);
////    auto res_num_points_per_voxel_t = torch::from_blob(res_num_points_per_voxel.data(), {43950, }).to(torch::kInt32);
////    auto res_voxel_point_mask_t = torch::from_blob(res_voxel_point_mask.data(), {43950,5, 1}).to(torch::kFloat32);
//
//
//    std::vector<int> num_filters = {16};
//    std::vector<float> voxel_size_vfe = {0.05, 0.05, 0.1};
//    std::vector<float> pc_range = {0. , -32. ,  -3. ,  52.8,  32. ,   1.};
//    auto fea = npy2tensor("/data/second_cpp/weights/vfe/vfe_features.npy");
//    auto vfe_num_voxel = npy2tensor("/data/second_cpp/weights/vfe/vfe_num_features.npy", torch::kCPU, true);
//    auto vfe_coors = npy2tensor("/data/second_cpp/weights/vfe/vfe_coors.npy", torch::kCPU, true);
//    SimpleVoxelRadius vfe = SimpleVoxelRadius(4, true, num_filters, false, voxel_size_vfe, pc_range);
//    auto vfe_res = vfe.forward(fea, vfe_num_voxel);
//    auto vfe_res2 = npy2tensor("/data/second_cpp/weights/vfe/vfe_res.npy");
//    std::cout<< " vfe dif is " << torch::max(torch::abs(vfe_res - vfe_res2)) << std::endl;
//}
void test_fuse(){
    // construct network
    auto input = test_middle();

    auto a1 = torch::nn::ZeroPad2d(torch::nn::ZeroPad2dOptions(1));
    auto conv = torch::nn::Conv2d(torch::nn::Conv2dOptions(128, 64, 3).stride(1).padding(0).groups(1).bias(false));
    auto bn = torch::nn::BatchNorm2d(torch::nn::BatchNorm2dOptions(64).eps(0.001).momentum(0.01));
    auto r = torch::nn::ReLU();
    torch::nn::Conv2dImpl *conv_imp = dynamic_cast<torch::nn::Conv2dImpl *>(conv.ptr().get());
    torch::nn::BatchNorm2dImpl *bn_imp = dynamic_cast<torch::nn::BatchNorm2dImpl *>(bn.ptr().get());
    // load weight
    auto w = npy2tensor("/data/second_cpp/weights/rpn/blocks_0_1_weight.npy");
    auto bn_w = npy2tensor("/data/second_cpp/weights/rpn/blocks_0_2_weight.npy");
    auto bn_b = npy2tensor("/data/second_cpp/weights/rpn/blocks_0_2_bias.npy");
    auto bn_m = npy2tensor("/data/second_cpp/weights/rpn/blocks_0_2_running_mean.npy");
    auto bn_v = npy2tensor("/data/second_cpp/weights/rpn/blocks_0_2_running_var.npy");
    std::cout<<"load weight shape : "<< w.sizes()<<std::endl;
    conv_imp->weight.set_data(w.view_as(conv_imp->weight));
    auto res = a1->forward(input); // after ZeroPad2d
    // test conv + bn
    auto input_f = res;
    conv->to(torch::kCUDA);
    conv->eval();
    bn->to(torch::kCUDA);
    // set weight bias
    bn->weight.set_data(bn_w.view_as(bn->weight));
    bn->bias.set_data(bn_b.view_as(bn->bias));
    bn->running_mean.set_data(bn_m.view_as(bn->running_mean));
    bn->running_var.set_data(bn_v.view_as(bn->running_var));
    bn->eval();

    //    std::cout<<"input : "<<res.sizes()<<std::endl;
    auto res1 = conv->forward(res);
    res1 = bn->forward(res1);
    std::cout<<"after conv : "<<res1.sizes()<<std::endl;
    std::cout<<"conv weight shape : "<<conv->weight.sizes()<<std::endl;

    // fuse bn; process weigt & bias
    auto var_sqrt = torch::sqrt(bn_v + 0.001).to(torch::kCUDA);
    auto bias = torch::zeros(bn_m.sizes()).to(torch::kCUDA);
    auto b_v = bn_w / var_sqrt;
    std::cout<<"fuse w size : "<<w.sizes()<<std::endl;
    for (int i = 0; i < w.size(0); ++i) {
        w[i] *= b_v[i];
    }
    bias = (bias - bn_m) / var_sqrt * bn_w + bn_b;

    auto conv_f = torch::nn::Conv2d(torch::nn::Conv2dOptions(128, 64, 3).stride(1).padding(0).groups(1).bias(true));
    torch::nn::Conv2dImpl *conv_imp_f = dynamic_cast<torch::nn::Conv2dImpl *>(conv_f.ptr().get());
    conv_imp_f->weight.set_data(w.view_as(conv_imp_f->weight));
    conv_imp_f->bias.set_data(bias.view_as(conv_imp_f->bias));
    conv_f->to(torch::kCUDA);
    conv_f->eval();
    auto res2 = conv_f->forward(input_f);
    std::cout<<"after fuse conv : "<<res2.sizes()<<std::endl;
    std::cout<<"fuse conv weight shape : "<<conv->weight.sizes()<<std::endl;


    auto dif = torch::abs(res1-res2);
    std::cout<<torch::max(dif)<<std::endl;
//    std::cout<<res1.index({0, 52, 122});
//    std::cout<<res2.index({0, 52, 122});
    std::cout<<"sucess!"<<std::endl;


//              w = conv.weight
//            mean = bn.running_mean
//            var_sqrt = torch.sqrt(bn.running_var + bn.eps)
//            beta = bn.weight
//            gamma = bn.bias
//            if conv.bias is not None:
//                  b = conv.bias
//            else:
//                  b = mean.new_zeros(mean.shape)
//            w = w * (beta / var_sqrt).reshape([conv.out_channels, 1, 1, 1])
//            b = (b - mean)/var_sqrt * beta + gamma
}
torch::Tensor test_spconv(){
    torch::Device device = torch::kCUDA;
    std::map<int, SparseConvolution> net_conv;
    std::vector<int64_t> k1 = {3,3,3}, k2 = {3,1,1};
    std::vector<int64_t> s1 = {1,1,1}, s2 = {2,2,2}, s3 = {2,1,1};
    std::vector<int64_t> p1 = {0,0,0}, p2 = {1,1,1}, p3 = {0,1,1};
    std::vector<int64_t> dil = {1,1,1}, out_p = {0,0,0};

    SparseConvolution conv1 = SparseConvolution(3, 3, 16, k1, s1, p1, dil, 1, true, out_p, false, false, "subm0");
    torch::Tensor weights = npy2tensor("/data/second_cpp/weights/weight.npy", device);
    torch::Tensor bias = npy2tensor("/data/second_cpp/weights/bias.npy", device);

    conv1.load_weights(weights, bias);
//    std::cout<< bias.sizes()[0] << std::endl;
    struct SparseConvTensor inputs;
    inputs.spatial_shape = {41, 1280, 1056};
    inputs.batch_size = 1;
    inputs.features = npy2tensor("/data/second_cpp/weights/feature.npy", device);
    inputs.indices = npy2tensor("/data/second_cpp/weights/indices.npy", torch::kCUDA, true);
//    std::cout<< inputs.features << std::endl;
    std::cout << "sucess !" << std::endl;
    SparseConvTensor outputs = conv1.forward(inputs);
    return outputs.features;
}
void test_rpn(){
    std::vector<int64_t> layer_nums = {5, 5};
    std::vector<int64_t> layer_strides = {1, 2};
    std::vector<int64_t> num_filters = {64, 128};
    std::vector<float> upsample_strides = {1.0, 2.0};
    std::vector<int64_t> num_upsample_filters = {128, 128};
    int64_t num_input_features = 128;
    int64_t num_anchor_per_loc = 8;
    bool encode_background_as_zeros = true;
    bool use_direction_classifier = true;
    bool use_groupnorm = false;
    int64_t num_groups = 32;
    int64_t box_code_size = 7;
    int64_t num_direction_bins = 2;
    auto x = torch::ones({1,1});
    // load weight
    std::map<std::string, torch::Tensor> weight;
    for (int i = 0; i < 2; i++){
        for (int j = 0; j < 18; ++j) {

            if (j % 3 == 0) {
                if (j == 0) {
                    std::string dconv_w = "db_" + std::to_string(i) + "_" + std::to_string(j) + "_w";
                    std::string dconv_w_f =
                            "/data/second_cpp/weights/rpn/deblocks_" + std::to_string(i) + "_" + std::to_string(j) +
                            "_weight.npy";
//                    std::cout<<dconv_w_f << std::endl;
                    weight[dconv_w] = npy2tensor(dconv_w_f);
                }
            }
            if (j % 3 == 1){
                std::string conv_w = "b_" + std::to_string(i) + "_" + std::to_string(j) + "_w";
                std::string conv_w_f = "/data/second_cpp/weights/rpn/blocks_" + std::to_string(i) + "_" + std::to_string(j) + "_weight.npy";
                weight[conv_w] = npy2tensor(conv_w_f);
                if (j == 1){
                    std::string dbn_w = "dbn_" + std::to_string(i) + "_" + std::to_string(j) + "_w";
                    std::string dbn_w_f = "/data/second_cpp/weights/rpn/deblocks_" + std::to_string(i) + "_" + std::to_string(j) + "_weight.npy";
                    std::string dbn_b = "dbn_" + std::to_string(i) + "_" + std::to_string(j) + "_b";
                    std::string dbn_b_f = "/data/second_cpp/weights/rpn/deblocks_" + std::to_string(i) + "_" + std::to_string(j) + "_bias.npy";
                    std::string dbn_m = "dbn_" + std::to_string(i) + "_" + std::to_string(j) + "_m";
                    std::string dbn_m_f = "/data/second_cpp/weights/rpn/deblocks_" + std::to_string(i) + "_" + std::to_string(j) + "_running_mean.npy";
                    std::string dbn_v = "dbn_" + std::to_string(i) + "_" + std::to_string(j) + "_v";
                    std::string dbn_v_f = "/data/second_cpp/weights/rpn/deblocks_" + std::to_string(i) + "_" + std::to_string(j) + "_running_var.npy";
                    std::string dbn_t = "dbn_" + std::to_string(i) + "_" + std::to_string(j) + "_t";
                    std::string dbn_t_f = "/data/second_cpp/weights/rpn/deblocks_" + std::to_string(i) + "_" + std::to_string(j) + "_num_batches_tracked.npy";
                    weight[dbn_w] = npy2tensor(dbn_w_f);
                    weight[dbn_b] = npy2tensor(dbn_b_f);
                    weight[dbn_m] = npy2tensor(dbn_m_f);
                    weight[dbn_v] = npy2tensor(dbn_v_f);
                    weight[dbn_t] = npy2tensor(dbn_t_f);
                }
            }
            if (j % 3 == 2){
                std::string bn_w = "bn_" + std::to_string(i) + "_" + std::to_string(j) + "_w";
                std::string bn_w_f = "/data/second_cpp/weights/rpn/blocks_" + std::to_string(i) + "_" + std::to_string(j) + "_weight.npy";
                std::string bn_b = "bn_" + std::to_string(i) + "_" + std::to_string(j) + "_b";
                std::string bn_b_f = "/data/second_cpp/weights/rpn/blocks_" + std::to_string(i) + "_" + std::to_string(j) + "_bias.npy";
                std::string bn_m = "bn_" + std::to_string(i) + "_" + std::to_string(j) + "_m";
                std::string bn_m_f = "/data/second_cpp/weights/rpn/blocks_" + std::to_string(i) + "_" + std::to_string(j) + "_running_mean.npy";
                std::string bn_v = "bn_" + std::to_string(i) + "_" + std::to_string(j) + "_v";
                std::string bn_v_f = "/data/second_cpp/weights/rpn/blocks_" + std::to_string(i) + "_" + std::to_string(j) + "_running_var.npy";
                std::string bn_t = "bn_" + std::to_string(i) + "_" + std::to_string(j) + "_t";
                std::string bn_t_f = "/data/second_cpp/weights/rpn/blocks_" + std::to_string(i) + "_" + std::to_string(j) + "_num_batches_tracked.npy";

                weight[bn_w] = npy2tensor(bn_w_f);
                weight[bn_b] = npy2tensor(bn_b_f);
                weight[bn_m] = npy2tensor(bn_m_f);
                weight[bn_v] = npy2tensor(bn_v_f);
                weight[bn_t] = npy2tensor(bn_t_f);
            }
        }
    }
    weight["cls_w"] = npy2tensor("/data/second_cpp/weights/rpn/conv_cls_weight.npy");
    weight["cls_b"] = npy2tensor("/data/second_cpp/weights/rpn/conv_cls_bias.npy");
    weight["box_w"] = npy2tensor("/data/second_cpp/weights/rpn/conv_box_weight.npy");
    weight["box_b"] = npy2tensor("/data/second_cpp/weights/rpn/conv_box_bias.npy");
    weight["dir_w"] = npy2tensor("/data/second_cpp/weights/rpn/conv_dir_cls_weight.npy");
    weight["dir_b"] = npy2tensor("/data/second_cpp/weights/rpn/conv_dir_cls_bias.npy");

    RPNV2 rpn = RPNV2(true, 4, layer_nums, layer_strides, num_filters, upsample_strides,
                      num_upsample_filters, num_input_features, num_anchor_per_loc,
                      encode_background_as_zeros, use_direction_classifier,
                      use_groupnorm, num_groups, box_code_size, num_direction_bins, false);
    auto input = test_middle();
//    rpn.load_weight(weight, true); // not fuse BN
    rpn.load_weight(weight, true, false);
    clock_t start = clock();
    std::map<std::string, torch::Tensor> res;
    res = rpn.forward(input);

    clock_t end = clock();
    double t = (double) (end - start) / CLOCKS_PER_SEC; // s
    std::cout<< "cost time : "<< t * 1000 << " ms!"<<std::endl;

    // dif
    auto res_d_p = res["dir_cls_preds"];
    auto res_d_o = npy2tensor("/data/second_cpp/weights/res/dir_cls_preds.npy");
//    std::cout << torch::max(res_d_o - res_d_p) << std::endl;
    auto res_b_p = res["box_preds"];
    auto res_b_o = npy2tensor("/data/second_cpp/weights/res/box_preds.npy");
//    std::cout << torch::max(res_b_p - res_b_o) << std::endl;
    auto res_c_p = res["cls_preds"];
    auto res_c_o = npy2tensor("/data/second_cpp/weights/res/cls_preds.npy");
//    std::cout << torch::max(torch::abs(res_c_p - res_c_o)) << std::endl;
    std::cout << "test rpn and middle2-net sucess !" << std::endl;
}


// =================================== construct inference module ===============================

VoxelGeneratorV2 construct_voxel_generator(){

    /** ============================= parameters ============================ **/
    std::vector<float> voxel_size = voxels; // voxel_size
    std::vector<float> point_cloud_range = point_cloud_ranges; // point_cloud_range
    int64_t max_num_points = 5;
    int64_t max_voxels = 20000;
    bool full_mean = false;
    bool block_filtering = false;
    int64_t block_factor = 0;
    int64_t block_size = 0;
    float height_threshold = 0.0;
    float height_high_threshold = 2.0;
    bool use_sigmoid_score = true;

    VoxelGeneratorV2  voxel_generate = VoxelGeneratorV2(voxel_size, point_cloud_range, max_num_points, max_voxels, full_mean,
                                                        block_filtering, block_factor, block_size, height_threshold, height_high_threshold);
    return voxel_generate;
}

VoxelNet construct_voxelnet(bool kd) {
    std::string weight_path = !kd ? "/data/second_cpp2/weights2" : "/data/second_cpp2/weights_kd";

    /** ============================= parameters ============================ **/
    std::vector<float> voxel_size = voxels;
    std::vector<float> point_cloud_range = point_cloud_ranges;

    // model_cfg.voxel_feature_extractor.num_filters
    std::vector<int> vfe_num_filters = {16};

    // model_cfg.voxel_feature_extractor.with_distance
    bool with_distance = false;

    // output_shape
    std::vector<int64_t> output_shape(5);
    std::vector<float> grid_size(3);
    for (int i = 0; i < 3; ++i) grid_size[i] = (point_cloud_range[i+3] - point_cloud_range[i]) / voxel_size[i];
    output_shape[0] = 1;
    for (int i = 0; i < 3; ++i) output_shape[i+1] = grid_size[2-i];
    output_shape[4] = vfe_num_filters[0];

    // nms_pre_max_sizes = list(model_cfg.target_assigner.nms_pre_max_sizes)
    // num of detect classes
    int num_class = 4;

    // nms_pre_max_size & nms_post_max_size & nms_score_threshold & nms_iou_threshold
    std::vector<int> nms_pre_max_sizes_all_class = {1000, 1000, 1000, 1000};  //pre
    std::vector<int> nms_post_max_sizes_all_class = {100, 100, 100, 100};  //pre
    std::vector<float> nms_iou_thresholds_all_class = {0.1, 0.1, 0.1, 0.1}; //pre
    std::vector<float> nms_score_thresholds = {0.30000001192092896, 0.30000001192092896, 0.30000001192092896, 0.30000001192092896}; // nms parameter
    int nms_pre_max_sizes =*std::max_element(nms_pre_max_sizes_all_class.begin(), nms_pre_max_sizes_all_class.end());
    int nms_post_max_sizes = *std::max_element(nms_post_max_sizes_all_class.begin(), nms_post_max_sizes_all_class.end());
    float nms_iou_thresholds = *std::max_element(nms_iou_thresholds_all_class.begin(), nms_iou_thresholds_all_class.end());

    //num_input_features = model_cfg.num_point_features
    int64_t num_input_features = 4;

    // model_cfg.middle_feature_extractor.num_input_features,
    int32_t middle_num_input_features = 3;


    // target_assigner.num_anchors_per_location. not in config!
    int64_t num_anchor_per_loc = 8;

    // model_cfg.rpn
    int64_t rpn_num_input_features = 128;
    std::vector<int64_t> rpn_layer_nums = {5, 5};
    std::vector<int64_t> rpn_layer_strides = {1, 2};            //rpn
    std::vector<int64_t> rpn_num_filters = {64, 128};           // rpn
    std::vector<float> rpn_upsample_strides = {1.0, 2.0};       // rpn
    std::vector<int64_t> rpn_num_upsample_filters = {128, 128}; // rpn
    bool use_groupnorm = false;
    int64_t num_groups = 32;

    // target_assigner.box_coder.code_size. not in config.
    // TODO : change it to target_assigner
    int64_t box_code_size = 7;

    // model_cfg
    bool use_direction_classifier = true;
    bool encode_background_as_zeros = true;
    int64_t num_direction_bins = 2;

    // Default parameters
    bool use_norm = true;

    struct Anchor_generators  s1 =  {{{0.0, -32.0, -1.0, 52.79999923760, 32.0, -1.0}},
                                     "Car",
                                     {0},
                                     {0.0, 1.570000052},
                                     {1.600000023841858, 3.9000000953674316, 1.559999942779541}};
    struct Anchor_generators  s2 =  {{{0.0, -32.0, -0.6000000238418579, 52.79999923706055, 32.0, -0.6000000238418579}},
                                     "Cyclist",
                                     {0},
                                     {0.0, 1.570000052},
                                     {0.6000000238418579, 1.7599999904632568, 1.7300000190734863}};
    struct Anchor_generators  s3 =  {{{0.0, -32.0, -0.6000000238418579, 52.79999923706055, 32.0, -0.6000000238418579}},
                                     "Pedestrain",
                                     {0},
                                     {0.0, 1.5700000524520874},
                                     {0.6000000238418579, 0.800000011920929, 1.7300000190734863}};
    struct Anchor_generators  s4 =  {{{0.0, -32.0, -1.409999966621399, 52.79999923706055, 32.0, -1.409999966621399}},
                                     "Van",
                                     {0},
                                     {0.0, 1.5700000524520874},
                                     {1.871037483215332, 5.028081893920898, 2.2096426486968994}};

    std::vector<struct Anchor_generators> anchor_generator;
    anchor_generator.push_back(s1);
    anchor_generator.push_back(s2);
    anchor_generator.push_back(s3);
    anchor_generator.push_back(s4);

    std::vector<std::vector<int>> feature_map_sizes = {{1, 160, 132},{1, 160, 132},{1, 160, 132},{1, 160, 132}};
    std::vector<int> feature_map_size = {1, 160, 132};


    /** ============================= weights ============================ **/
    std::map<std::string, std::map<std::string, torch::Tensor>> weights;
    std::map<std::string, torch::Tensor> weight_rpn;
    // rpn weight
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 18; ++j) {

            if (j % 3 == 0) {
                if (j == 0) {
                    std::string dconv_w = "db_" + std::to_string(i) + "_" + std::to_string(j) + "_w";
                    std::string dconv_w_f =
                            weight_path + "/rpn/deblocks_" + std::to_string(i) + "_" + std::to_string(j) +
                            "_weight.npy";
                    //                    std::cout<<dconv_w_f << std::endl;
                    weight_rpn[dconv_w] = npy2tensor(dconv_w_f);
                }
            }
            if (j % 3 == 1) {
                std::string conv_w = "b_" + std::to_string(i) + "_" + std::to_string(j) + "_w";
                std::string conv_w_f =
                        weight_path + "/rpn/blocks_" + std::to_string(i) + "_" + std::to_string(j) +
                        "_weight.npy";
                weight_rpn[conv_w] = npy2tensor(conv_w_f);
                if (j == 1) {
                    std::string dbn_w = "dbn_" + std::to_string(i) + "_" + std::to_string(j) + "_w";
                    std::string dbn_w_f =
                            weight_path + "/rpn/deblocks_" + std::to_string(i) + "_" + std::to_string(j) +
                            "_weight.npy";
                    std::string dbn_b = "dbn_" + std::to_string(i) + "_" + std::to_string(j) + "_b";
                    std::string dbn_b_f =
                            weight_path + "/rpn/deblocks_" + std::to_string(i) + "_" + std::to_string(j) +
                            "_bias.npy";
                    std::string dbn_m = "dbn_" + std::to_string(i) + "_" + std::to_string(j) + "_m";
                    std::string dbn_m_f =
                            weight_path + "/rpn/deblocks_" + std::to_string(i) + "_" + std::to_string(j) +
                            "_running_mean.npy";
                    std::string dbn_v = "dbn_" + std::to_string(i) + "_" + std::to_string(j) + "_v";
                    std::string dbn_v_f =
                            weight_path + "/rpn/deblocks_" + std::to_string(i) + "_" + std::to_string(j) +
                            "_running_var.npy";
                    std::string dbn_t = "dbn_" + std::to_string(i) + "_" + std::to_string(j) + "_t";
                    std::string dbn_t_f =
                            weight_path + "/rpn/deblocks_" + std::to_string(i) + "_" + std::to_string(j) +
                            "_num_batches_tracked.npy";
                    weight_rpn[dbn_w] = npy2tensor(dbn_w_f);
                    weight_rpn[dbn_b] = npy2tensor(dbn_b_f);
                    weight_rpn[dbn_m] = npy2tensor(dbn_m_f);
                    weight_rpn[dbn_v] = npy2tensor(dbn_v_f);
                    weight_rpn[dbn_t] = npy2tensor(dbn_t_f);
                }
            }
            if (j % 3 == 2) {
                std::string bn_w = "bn_" + std::to_string(i) + "_" + std::to_string(j) + "_w";
                std::string bn_w_f =
                        weight_path + "/rpn/blocks_" + std::to_string(i) + "_" + std::to_string(j) +
                        "_weight.npy";
                std::string bn_b = "bn_" + std::to_string(i) + "_" + std::to_string(j) + "_b";
                std::string bn_b_f =
                        weight_path + "/rpn/blocks_" + std::to_string(i) + "_" + std::to_string(j) +
                        "_bias.npy";
                std::string bn_m = "bn_" + std::to_string(i) + "_" + std::to_string(j) + "_m";
                std::string bn_m_f =
                        weight_path + "/rpn/blocks_" + std::to_string(i) + "_" + std::to_string(j) +
                        "_running_mean.npy";
                std::string bn_v = "bn_" + std::to_string(i) + "_" + std::to_string(j) + "_v";
                std::string bn_v_f =
                        weight_path + "/rpn/blocks_" + std::to_string(i) + "_" + std::to_string(j) +
                        "_running_var.npy";
                std::string bn_t = "bn_" + std::to_string(i) + "_" + std::to_string(j) + "_t";
                std::string bn_t_f =
                        weight_path + "/rpn/blocks_" + std::to_string(i) + "_" + std::to_string(j) +
                        "_num_batches_tracked.npy";

                weight_rpn[bn_w] = npy2tensor(bn_w_f);
                weight_rpn[bn_b] = npy2tensor(bn_b_f);
                weight_rpn[bn_m] = npy2tensor(bn_m_f);
                weight_rpn[bn_v] = npy2tensor(bn_v_f);
                weight_rpn[bn_t] = npy2tensor(bn_t_f);
            }
        }
    }
    weight_rpn["cls_w"] = npy2tensor(weight_path + "/rpn/conv_cls_weight.npy");
    weight_rpn["cls_b"] = npy2tensor(weight_path + "/rpn/conv_cls_bias.npy");
    weight_rpn["box_w"] = npy2tensor(weight_path + "/rpn/conv_box_weight.npy");
    weight_rpn["box_b"] = npy2tensor(weight_path + "/rpn/conv_box_bias.npy");
    weight_rpn["dir_w"] = npy2tensor(weight_path + "/rpn/conv_dir_cls_weight.npy");
    weight_rpn["dir_b"] = npy2tensor(weight_path + "/rpn/conv_dir_cls_bias.npy");
    std::map<std::string, torch::Tensor> weights_mid;
    torch::Tensor bias = torch::zeros({0}).to(torch::kCUDA);

    // midlle weight
    for (int i = 0; i < (!kd ? 14 : 8); i++) {
        weights_mid["w" + std::to_string(i)] = npy2tensor(
                weight_path + "/middle2/conv" + std::to_string(i) + ".npy");
        weights_mid["b" + std::to_string(i)] = bias;
        weights_mid["bn_w" + std::to_string(i)] = npy2tensor(
                weight_path + "/middle2/bn" + std::to_string(i) + "_w.npy");
        weights_mid["bn_b" + std::to_string(i)] = npy2tensor(
                weight_path + "/middle2/bn" + std::to_string(i) + "_b.npy");
        weights_mid["bn_m" + std::to_string(i)] = npy2tensor(
                weight_path + "/middle2/bn" + std::to_string(i) + "_mean.npy");
        weights_mid["bn_v" + std::to_string(i)] = npy2tensor(
                weight_path + "/middle2/bn" + std::to_string(i) + "_var.npy");
    }

    weights["middle2"] = weights_mid;
    weights["rpn"] = weight_rpn;

    VoxelNet voxelNet = VoxelNet(output_shape, voxel_size, point_cloud_range,
//                                 max_num_points, max_voxels,full_mean,
//                                 block_filtering, block_factor, block_size, height_threshold,height_high_threshold,
                                 weights,
//                                 use_sigmoid_score,
                                 nms_pre_max_sizes,nms_post_max_sizes,nms_iou_thresholds, nms_score_thresholds,
                                 anchor_generator,feature_map_sizes,feature_map_size,
                                 num_class, num_input_features, vfe_num_filters, with_distance,
                                 middle_num_input_features,
                                 rpn_num_input_features, num_anchor_per_loc, rpn_layer_nums, rpn_layer_strides,
                                 rpn_num_filters,
                                 rpn_upsample_strides, rpn_num_upsample_filters, use_norm, use_groupnorm, num_groups,
                                 box_code_size,
                                 use_direction_classifier, encode_background_as_zeros, num_direction_bins, kd);
    return voxelNet;
}

