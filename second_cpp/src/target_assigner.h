//
// Created by root on 4/7/21.
//

#ifndef SECOND_DETECTOR_TARGET_ASSIGNER_H
#define SECOND_DETECTOR_TARGET_ASSIGNER_H

#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <ctime>
#include <torch/script.h>
#include <xtensor/xarray.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xpad.hpp>
#include <xtensor/xbuilder.hpp>
#include <xtensor/xmanipulation.hpp>


struct Anchor_generators{
    xt::xarray<float> _anchor_ranges;
    std::string _class_name;
    xt::xarray<float> _custom_values;
    xt::xarray<float> _rotations;
    xt::xarray<float> _sizes;
};



class TargetAssigner{
public:

    TargetAssigner();

    TargetAssigner(std::vector<struct Anchor_generators> anchor_generator,
                   std::vector<std::vector<int>> feature_map_sizes);

    ~TargetAssigner();

    int box_ndim();

    torch::Tensor generate_anchors(const std::vector<int> & feature_map_size);

    std::vector<struct Anchor_generators> get_anchor_generator();

    void set_anchor_generator(struct Anchor_generators);

    std::vector<std::vector<int>> get_feature_map_sizes();

private:

    std::vector<struct Anchor_generators> _anchor_generator;
    std::vector<std::vector<int>> _feature_map_sizes;




};

#endif //SECOND_DETECTOR_TARGET_ASSIGNER_H
