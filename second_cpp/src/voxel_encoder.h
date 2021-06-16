//
// Created by root on 3/16/21.
//

#ifndef SECOND_DETECTOR_VOXEL_ENCODER_H
#define SECOND_DETECTOR_VOXEL_ENCODER_H
#include <iostream>
#include <vector>
#include <map>
#include <torch/script.h>
#include <torch/torch.h>
#include <xtensor/xarray.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xview.hpp>
class VoxelGeneratorV2{
public:
    VoxelGeneratorV2(std::vector<float> voxel_size,
                     std::vector<float> point_cloud_range,
                     int64_t max_num_points,
                     int64_t max_voxels,
                     bool full_mean,
                     bool block_filtering,
                     int64_t block_factor,
                     int64_t block_size,
                     float height_threshold,
                     float height_high_threshold);
    VoxelGeneratorV2();
//    std::map<std::string, torch::Tensor> generate(xt::xarray<float> &points, int64_t max_voxels);
//    std::map<std::string, torch::Tensor> points_to_voxel(xt::xarray<float> &points,
//                                                         std::vector<float> &voxel_size,
//                                                         std::vector<float> &point_cloud_range,
//                                                         xt::xarray<int> &coor_to_voxelidx,
//                                                         int64_t &max_points,
//                                                         int64_t &max_voxels,
//                                                         bool full_mean,
//                                                         bool block_filtering,
//                                                         int64_t block_factor,
//                                                         int64_t block_size,
//                                                         float height_threshold,
//                                                         float height_high_threshold,
//                                                         bool pad_output = false);
    void points_to_voxel(xt::xarray<float> &points, int64_t max_voxels,
                                  xt::xarray<int> &num_points_per_voxel,
                                  xt::xarray<float> &voxels,
                                  xt::xarray<float> &voxel_point_mask,
                                  xt::xarray<int> &coors);
    std::vector<float> _voxel_size;
private:
    xt::xarray<int> _coor_to_voxelidx;

    std::vector<float> _point_cloud_range;
    int64_t _max_num_points;
    int64_t _max_voxels;
    torch::Tensor _grid_size;
    bool _full_mean;
    bool _block_filtering;
    int64_t _block_factor;
    int64_t _block_size;
    float _height_threshold;
    float _height_high_threshold;
};

class SimpleVoxelRadius{
public:
    SimpleVoxelRadius(int64_t num_input_features, bool use_norm, std::vector<int> num_filters,
                      bool with_distance, std::vector<float> voxel_size, std::vector<float> pc_range);
    SimpleVoxelRadius();
    torch::Tensor forward(torch::Tensor features, torch::Tensor num_voxels);
    torch::Tensor forward_xt(xt::xarray<float> features, xt::xarray<int> num_voxels);

private:
    int64_t _num_input_features;
};
#endif //SECOND_DETECTOR_VOXEL_ENCODER_H
