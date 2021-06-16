//
// Created by root on 4/7/21.
//

#include "target_assigner.h"
#include <torch/script.h>



xt::xarray<float> create_anchors_3d_range(std::vector<int> fsize,
                                          xt::xarray<float> _anchor_range,
                                          xt::xarray<float> _sizes,
                                          xt::xarray<float> _rotations){

//    std::cout << _rotations << std::endl;
//    std::cout << _sizes << std::endl;
    xt::xarray<float> anchor_range = _anchor_range.reshape({-1});

    xt::xarray<float> z_centers = xt::linspace(anchor_range[2],
                                               anchor_range[5],
                                               fsize[0],
                                               true);
    xt::xarray<float> y_centers = xt::linspace(anchor_range[1],
                                               anchor_range[4],
                                               fsize[1],
                                               true);
    xt::xarray<float> x_centers = xt::linspace(anchor_range[0],
                                               anchor_range[3],
                                               fsize[2],
                                               true);
    xt::xarray<float> sizes = _sizes.reshape({-1, 3});
    xt::xarray<float> rotations =_rotations;
    auto rets = xt::meshgrid(x_centers, y_centers, z_centers, rotations);
    int tmp = std::get<0>(rets).dimension();
    auto tmp1 = std::get<0>(rets);
    auto tmp2 = std::get<1>(rets);
    auto tmp3 = std::get<2>(rets);
    auto tmp4 = std::get<3>(rets);

    std::vector<size_t> title_shape = {1,1,1,1,1};
    title_shape[3] = int(sizes.shape(0));

    xt::xarray<float> a = xt::tile(xt::strided_view(tmp1,{xt::ellipsis(),xt::newaxis(),xt::all()}),title_shape);
    a = xt::strided_view(a, {xt::ellipsis(), xt::newaxis()});

    xt::xarray<float> b = xt::tile(xt::strided_view(tmp2,{xt::ellipsis(),xt::newaxis(),xt::all()}),title_shape);
    b = xt::strided_view(b, {xt::ellipsis(), xt::newaxis()});

    xt::xarray<float> c = xt::tile(xt::strided_view(tmp3,{xt::ellipsis(),xt::newaxis(),xt::all()}),title_shape);
    c = xt::strided_view(c, {xt::ellipsis(), xt::newaxis()});

    xt::xarray<float> d = xt::tile(xt::strided_view(tmp4,{xt::ellipsis(),xt::newaxis(),xt::all()}),title_shape);
    d = xt::strided_view(d, {xt::ellipsis(), xt::newaxis()});

    std::vector<xt::xarray<float>> rets_2;
    rets_2.push_back(a);
    rets_2.push_back(b);
    rets_2.push_back(c);


    sizes = sizes.reshape({1, 1, 1, -1, 1, 3});
    auto tile_size_shape = a.shape();
    tile_size_shape[3] = 1;
    sizes = xt::tile(sizes, tile_size_shape);

    rets_2.push_back(sizes);
    rets_2.push_back(d);

    xt::xarray<float> result = xt::concatenate(xt::xtuple(a, b, c, sizes, d),5);
    xt::xarray<float> resutl2 = xt::transpose(result,{2, 1, 0, 3, 4, 5});

    return resutl2;

}

xt::xarray<float> generate(std::vector<int> fsize,
                           struct Anchor_generators anchor_generator){
    xt::xarray<float> res =  create_anchors_3d_range(fsize,
                                                     anchor_generator._anchor_ranges ,
                                                     anchor_generator._sizes,
                                                     anchor_generator._rotations);

    if (anchor_generator._custom_values.size() > 0)       //not use
    {}                                                    //not use
    return res;
}


TargetAssigner::TargetAssigner(){
    _anchor_generator = {};
    _feature_map_sizes = {};
}


TargetAssigner::TargetAssigner(std::vector<struct Anchor_generators> anchor_generator,
                               std::vector<std::vector<int>> feature_map_sizes)
{
    _anchor_generator = anchor_generator;
    _feature_map_sizes = feature_map_sizes;
}


TargetAssigner::~TargetAssigner(){}


int TargetAssigner::box_ndim(){
    return 0;
}

torch::Tensor TargetAssigner::generate_anchors(const std::vector<int> & feature_map_size){
    std::vector<xt::xarray<float>> anchors_list;
    std::vector<std::vector<int>> feature_map_sizes;

    int ndim = feature_map_size.size();
    if (!_feature_map_sizes.empty())
    {
        feature_map_sizes = _feature_map_sizes;
    }else
    {
//        feature_map_sizes = [feature_map_size] * len(self._anchor_generators)
    }
    int idx = 0;
    struct Anchor_generators anchor_generator;
    xt::xarray<float> anchors;
    std::vector<int> fsize = _feature_map_sizes[0];
    int loop = feature_map_sizes.size();
    for (int i = 0; i < loop; ++i) {
        anchor_generator = _anchor_generator[i];
        if (fsize.empty())
        {
            fsize = feature_map_size;
            _feature_map_sizes[idx] = feature_map_size;
        }
//        anchors = anchor_generator.generate(fsize)
        anchors =  generate(fsize, anchor_generator);
        anchors = anchors.reshape({fsize[0], fsize[1], fsize[2], -1 , 7});   //  *fsize   7 = self.box_ndim
        anchors = xt::transpose(anchors, {ndim, 0,1,2, ndim+1});      // *range(0, ndim)
        anchors_list.push_back(anchors.reshape({-1,7}));     //7 = self.box_ndim

    }
    anchors = xt::concatenate(xt::xtuple(anchors_list[0],anchors_list[1],anchors_list[2],anchors_list[3]),0);

//    std::cout << anchors <<std::endl;
    anchors.reshape({1,-1,7});
    xt::xarray<int> a = xt::adapt(anchors.shape());
    torch::Tensor anchor = torch::from_blob(anchors.data(), {a(0),a(1),a(2)}, torch::kFloat).to(torch::kCUDA);
    return anchor;
}

std::vector<struct Anchor_generators> TargetAssigner::get_anchor_generator(){

    return _anchor_generator;
}

void TargetAssigner::set_anchor_generator(struct Anchor_generators s){
    _anchor_generator.push_back(s);
}

std::vector<std::vector<int>> TargetAssigner::get_feature_map_sizes(){
    return _feature_map_sizes;
}

