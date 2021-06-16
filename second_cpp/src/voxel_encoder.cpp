//
// Created by root on 3/16/21.
//

#include "voxel_encoder.h"
#include <vector>
#include <torch/script.h>
#include <cmath>
#include "ops.h"
#include <xtensor/xbuilder.hpp>
#include "xtensor/xview.hpp"


void VoxelGeneratorV2::points_to_voxel(xt::xarray<float> &points, int64_t max_voxels,
                                                xt::xarray<int> &num_points_per_voxel,
                                                xt::xarray<float> &voxels,
                                                xt::xarray<float> &voxel_point_mask,
                                                xt::xarray<int> &coors) {

    xt::xarray<int> coors_pad = xt::zeros<int>({max_voxels, int64_t(1)});
//    auto t1 = std::chrono::steady_clock::now();
    int voxel_num = ops::points_to_voxel_XT<float, 3>(points, voxels, voxel_point_mask,
                                                    coors, num_points_per_voxel,
                                                    _coor_to_voxelidx,
                                                    _voxel_size,
                                                    _point_cloud_range,
                                                    _max_num_points,
                                                    max_voxels);


    coors = xt::view(xt::concatenate(xt::xtuple(coors_pad, coors), 1), xt::range(0, voxel_num), xt::all());
    voxels = xt::view(voxels, xt::range(0, voxel_num));
    num_points_per_voxel = xt::view(num_points_per_voxel, xt::range(0, voxel_num));
    voxel_point_mask = xt::view(voxel_point_mask, xt::range(0, voxel_num));
//    auto t2 = std::chrono::steady_clock::now();
//    double vox = std::chrono::duration<double, std::milli>(t2 - t1).count();
//    std::cout<<vox<<std::endl;
}



VoxelGeneratorV2::VoxelGeneratorV2(std::vector<float> voxel_size, std::vector<float> point_cloud_range,
                                   int64_t max_num_points, int64_t max_voxels, bool full_mean, bool block_filtering,
                                   int64_t block_factor, int64_t block_size, float height_threshold,
                                   float height_high_threshold):
                                   _voxel_size(voxel_size),
                                   _point_cloud_range(point_cloud_range),
                                   _max_num_points(max_num_points),
                                   _max_voxels(max_voxels),
                                   _full_mean(full_mean),
                                   _block_filtering(block_filtering),
                                   _block_factor(block_factor),
                                   _height_threshold(height_threshold),
                                   _block_size(block_size),
                                   _height_high_threshold(height_high_threshold){
    assert(full_mean == false);
    std::vector<float> grid_size(3);
    for (int i = 0; i < 3; ++i) {
        grid_size[i] = (point_cloud_range[i+3] - point_cloud_range[i]) / voxel_size[i];
    }
//    if (block_filtering){
//        assert(block_size > 0);
////        assert(grid_size[0] % block_factor == 0);
////        assert(grid_size[1] % block_factor == 0);
//    }
//    std::cout<<grid_size<<std::endl;
//    std::cout<< grid_size<<std::endl;
    _coor_to_voxelidx = xt::ones<int>({(int) grid_size[2],(int)grid_size[1], (int) grid_size[0]}) * (-1);
//    std::cout<<_coor_to_voxelidx<<std::endl;
//    _coor_to_voxelidx = xt::full_like({(int) grid_size[2],(int)grid_size[1], (int) grid_size[0]}, -1).to(torch::kInt32);
    _grid_size = torch::round(torch::tensor(grid_size).to(torch::kFloat32)).to(torch::kInt64);
}

VoxelGeneratorV2::VoxelGeneratorV2() {}

//std::map<std::string, torch::Tensor> VoxelGeneratorV2::generate(xt::xarray<float> &points, int64_t max_voxels) {
//
////    auto res = points_to_voxel(points, _voxel_size, _point_cloud_range, _coor_to_voxelidx, _max_num_points, max_voxels, _full_mean,
////                                                 _block_filtering, _block_factor, _block_size, _height_threshold, _height_high_threshold);
////    std::cout<<res["voxels"]<<std::endl;
////    return res;
//}

SimpleVoxelRadius::SimpleVoxelRadius(int64_t num_input_features, bool use_norm, std::vector<int> num_filters,
                                     bool with_distance, std::vector<float> voxel_size, std::vector<float> pc_range):
                                     _num_input_features(num_input_features){
}

SimpleVoxelRadius::SimpleVoxelRadius() {}
torch::Tensor SimpleVoxelRadius::forward(torch::Tensor features, torch::Tensor num_voxels) {
    std::cout<<num_voxels<<std::endl;
    auto points_mean =
            features.slice(2, 0, _num_input_features).sum(1, false) / num_voxels.type_as(features).view({-1, 1});
    features = torch::norm(points_mean.slice(1, 0, 2), 2, 1, true);
    torch::Tensor res = torch::cat({features, points_mean.slice(1, 2, _num_input_features)}, 1);

    res = torch::where(torch::isnan(res), torch::full_like(res, 0), res);
    res = torch::where(torch::isinf(res), torch::full_like(res, 0), res);



//    std::cout<< res.sizes() << "\n";
//    std::cout<<"max is : \n" << torch::max(res).item<float>() << "\n";
//    std::cout<<"min is : \n" << torch::min(res).item<float>() << "\n";

    return res;
}


torch::Tensor SimpleVoxelRadius::forward_xt(xt::xarray<float> features, xt::xarray<int> num_voxels) {
    xt::xarray<int> fea_shape = xt::adapt(features.shape());
    torch::Tensor features_t = torch::from_blob(features.data(), {fea_shape(0), fea_shape(1), fea_shape(2)}, torch::kFloat32).to(torch::kCUDA);
    xt::xarray<int> num_shape = xt::adapt(num_voxels.shape());
    torch::Tensor num_voxels_t = torch::from_blob(num_voxels.data(), {num_shape(0), }, torch::kInt32).to(torch::kCUDA);


    auto points_mean =
            features_t.slice(2, 0, _num_input_features).sum(1, false) / num_voxels_t.type_as(features_t).view({-1, 1});
    features_t = torch::norm(points_mean.slice(1, 0, 2), 2, 1, true);
    torch::Tensor res = torch::cat({features_t, points_mean.slice(1, 2, _num_input_features)}, 1);

//    res = torch::where(torch::isnan(res), torch::full_like(res, 0), res);
//    res = torch::where(torch::isinf(res), torch::full_like(res, 0), res);
    std::cout<<res.sizes() << std::endl;
    return res;
}
