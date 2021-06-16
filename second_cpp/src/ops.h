//
// Created by root on 3/4/21.
//

#ifndef SPARSECONVOLUTIONPLUGIN_OPS_H
#define SPARSECONVOLUTIONPLUGIN_OPS_H


#include <iostream>
#include <torch/script.h>
#include <torch/torch.h>
#include <vector>
#include <string>
#include <map>
#include <assert.h>
#include <xtensor/xarray.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xview.hpp>

namespace ops{
    template <typename DType, int NDim=3>
    int points_to_voxel_XT(xt::xarray<float> &points,
                              xt::xarray<float> &voxels,
                              xt::xarray<float> &voxel_point_mask,
                              xt::xarray<int> &coors,
                              xt::xarray<int> &num_points_per_voxel,
                              xt::xarray<int> &coor_to_voxelidx,
                              std::vector<float> &voxel_size,
                              std::vector<float> &coors_range,
                              int64_t &max_points,
                              int64_t &max_voxels){
        auto N = points.shape(0);
        auto num_features = points.shape(1);
        constexpr int ndim_minus_1 = 3 - 1;
        int voxel_num = 0;
        bool failed = false;
        int coor[3];
        int c;
        int grid_size[3];
        for (int i = 0; i < 3; ++i) {
            grid_size[i] = round((coors_range[3 + i] - coors_range[i]) / voxel_size[i]);
        }
        int voxelidx, num;
        for (int i = 0; i < N; ++i) {
            failed = false;
            for (int j = 0; j < 3; ++j) {
                c = floor((points(i, j) - coors_range[j]) / voxel_size[j]);
                if (c < 0 || c >= grid_size[j]){
                    failed = true;
                    break;
                }
                coor[ndim_minus_1 -j] = c;
            }
            if (failed)
                continue;
            voxelidx = coor_to_voxelidx(coor[0], coor[1], coor[2]);
            if (voxelidx == -1){
                voxelidx = voxel_num;
                if (voxel_num >= max_voxels) continue;
                voxel_num += 1;
                coor_to_voxelidx(coor[0], coor[1], coor[2]) = voxelidx;
                for (int k = 0; k < 3; ++k)
                    coors(voxelidx, k) = coor[k];
            }
            num = num_points_per_voxel(voxelidx);
            if (num < max_points){
                voxel_point_mask(voxelidx, num) = float(1);
                for (int k = 0; k < num_features; ++k)
                    voxels(voxelidx, num, k) = points(i, k);
                auto x = num_points_per_voxel(voxelidx);
                num_points_per_voxel(voxelidx) = x+1;
            }
        }
        for (int i = 0; i < voxel_num; ++i)
            coor_to_voxelidx(coors(i, 0), coors(i, 1), coors(i, 2)) = -1;
        return voxel_num;
    }
    void dense(torch::Tensor &features, std::vector<int64_t> spatial_shape, torch::Tensor &indices, int batch_size);

    std::vector<torch::Tensor> get_indice_pairs(torch::Tensor indices,
                                                int64_t batch_size,
                                                 std::vector<int64_t> spatial_shape,
                                                 std::vector<int64_t> ksize,
                                                 std::vector<int64_t> stride,
                                                 std::vector<int64_t> padding,
                                                 std::vector<int64_t> dilation,
                                                 std::vector<int64_t> out_padding,
                                                 bool subm,
                                                 bool transpose,
                                                 std::vector<int64_t> grid,
                                                 bool use_hash);
    torch::Tensor indice_conv(torch::Tensor features,
                              torch::Tensor filters,
                              torch::Tensor indice_pairs,
                              torch::Tensor indice_pair_num,
                              int64_t num_activate_out,
                              bool inverse,
                              bool subm,
                              int64_t algo);
    std::vector<int64_t> get_conv_output_size(std::vector<int64_t> &input_size,
                                              std::vector<int64_t> &kernel_size,
                                              std::vector<int64_t> &stride,
                                              std::vector<int64_t> &padding,
                                              std::vector<int64_t> &dilation);
    torch::Tensor scatter_nd(torch::Tensor &indice, torch::Tensor &features, std::vector<int64_t> &out_shape);

}
#endif //SPARSECONVOLUTIONPLUGIN_OPS_H