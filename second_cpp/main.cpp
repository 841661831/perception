//
// Created by chenxiangyang on 3/1/21.
//



#include <iostream>
#include <string>
#include <xtensor/xarray.hpp>
#include <xtensor/xnpy.hpp>
#include "inference.h"



int main(int argc, char* argv[]){

    printf("====================== SECOND-detector start ====================\n");
    at::Tensor a = torch::zeros({9000, 5, 4});

    auto voxel_generator = construct_voxel_generator();
    auto voxelnet = construct_voxelnet(true);
    std::string velo_path = "/data/second_cpp2/datasets/kitti/";

    double t = 0;
    for (int i = 0; i < 21; ++i) {
        std::string path = i >= 10 ? velo_path + "0000" + std::to_string(i) + ".npy" : velo_path + "00000" + std::to_string(i) + ".npy";
//        path = velo_path + "000001.npy";

        xt::xarray<int> num_points_per_voxel = xt::zeros<int>({90000,});
        xt::xarray<float> voxels = xt::zeros<float>({90000, 5, 4});
        xt::xarray<float> voxel_point_mask = xt::zeros<float>({90000, 5});
        xt::xarray<int> coors = xt::zeros<int>({90000, 3});

        xt::xarray<float> points = xt::load_npy<float>(path);
        points.reshape({-1, 4});
//        points = xt::view(points, xt::range(0, 60000), xt::all());
        auto t1 = std::chrono::steady_clock::now();
        voxel_generator.points_to_voxel(points, 90000, num_points_per_voxel, voxels, voxel_point_mask, coors);
        auto t2 = std::chrono::steady_clock::now();
        double vox = std::chrono::duration<double, std::milli>(t2 - t1).count();
//        t += vox;
        printf("vox cost %4f ms!\n", vox);
//        std::cout<<voxels<<"\n";
        auto res = voxelnet.forward(voxels, num_points_per_voxel, coors);

        /** ================================= save res to npy ============================= **/

        /*size_t tensorSize = (size_t) res["box3d_lidar"].size(0) * res["box3d_lidar"].size(1);
        std::vector<int> tensorShape = {int(res["box3d_lidar"].size(0)),
                                        int(res["box3d_lidar"].size(1))};
        xt::xarray<float> outXtensorItem = xt::adapt(res["box3d_lidar"].to(torch::kCPU).data_ptr<float>(),
                                                     tensorSize,
                                                     xt::no_ownership(),
                                                     tensorShape);

        std::string name = i >= 10 ? "0000" + std::to_string(i) + "_cpp_kd.npy" : "00000" + std::to_string(i) + "_cpp_kd.npy";
        xt::dump_npy("/data/second_cpp2/datasets/pre_res/" + name, outXtensorItem);*/


//        std::cout << res["box3d_lidar"] << "\n";
//        std::cout << res["scores"] << "\n";
//        std::cout << res["label_preds"] << "\n";

        std::cout << "======================= SECOND-detector " << i << "  =====================\n";
    }
//    printf("average cost time : %4f ms!", t / 100);
    return 0;
}

