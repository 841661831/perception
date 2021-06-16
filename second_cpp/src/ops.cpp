//
// Created by root on 3/4/21.
//

#include "ops.h"

#include <torch/script.h>
#include <vector>
//#include <xtensor/xarray.hpp>


#include "../include/spconv/spconv_ops.h"
#include<ctime>
#include <math.h>


std::vector<torch::Tensor> ops::get_indice_pairs(torch::Tensor indices, int64_t batch_size,
                                                 std::vector<int64_t> spatial_shape, std::vector<int64_t> ksize,
                                                 std::vector<int64_t> stride, std::vector<int64_t> padding,
                                                 std::vector<int64_t> dilation, std::vector<int64_t> out_padding, bool subm,
                                                 bool transpose, std::vector<int64_t> grid, bool use_hash) {
    std::vector<int64_t> out_shape;
    std::vector<torch::Tensor> res;
    int ndim = indices.sizes()[1] - 1;
    if (!subm){
        if (transpose){
//            out_shape = get_deconv_output_size(spatial_shape, ksize, stride,
//                                               padding, dilation, out_padding)
        } else{
            out_shape = ops::get_conv_output_size(spatial_shape, ksize, stride, padding, dilation);
//            out_shape = get_conv_output_size(spatial_shape, ksize, stride,
//                                             padding, dilation)
        }
    } else{
        out_shape = spatial_shape;
    }
//    std::cout<<indices.sizes();

//    std::cout<< "batch_size " << batch_size << std::endl;
//    std::cout<< "out_shape " << out_shape << std::endl;
//    std::cout<< "ksize " << ksize << std::endl;
//    std::cout<< "stride " << stride << std::endl;
//    std::cout<< "padding " << padding << std::endl;
//    std::cout<< "dilation " << dilation << std::endl;
//    std::cout<< "out_padding " << out_padding << std::endl;
//    std::cout<< "subm " << (int) subm << std::endl;
//    std::cout<< "transpose " << (int) transpose << std::endl;
//    std::cout<< "use_hash " << (int) use_hash << std::endl;
//    std::cout<<indices.slice(0, 100, 105)<< std::endl;
    indices = torch::where(torch::isnan(indices), torch::full_like(indices, 0), indices);
    indices = torch::where(torch::isinf(indices), torch::full_like(indices, 0), indices);

    res = spconv::getIndicePairs(indices,
                                 batch_size,
                                 out_shape,
                                 spatial_shape,
                                 ksize,
                                 stride,
                                 padding,
                                 dilation,
                                 out_padding,
                                 (int) subm,
                                 (int) transpose,
                                 (int) use_hash);
    return res;
}

torch::Tensor ops::indice_conv(torch::Tensor features, torch::Tensor filters, torch::Tensor indice_pairs,
                               torch::Tensor indice_pair_num, int64_t num_activate_out, bool inverse, bool subm,
                               int64_t algo) {
//    std::cout << "features " << features << std::endl;
//    std::cout << "filters " << filters << std::endl;
//    std::cout << "indice_pairs " << indice_pairs << std::endl;
//    std::cout << "indice_pair_num " << indice_pair_num << std::endl;
//    std::cout << "num_activate_out " << num_activate_out << std::endl;
//    std::cout << "inverse " << (int64_t) inverse << std::endl;
//    std::cout << "subm " << (int64_t) subm << std::endl;
//    std::cout << "algo " << algo << std::endl;
    return spconv::indiceConv(features, filters, indice_pairs, indice_pair_num,
                              num_activate_out, (int64_t) inverse, (int64_t) subm, algo);
}

std::vector<int64_t> ops::get_conv_output_size(std::vector<int64_t> &input_size, std::vector<int64_t> &kernel_size,
                                               std::vector<int64_t> &stride,
                                               std::vector<int64_t> &padding, std::vector<int64_t> &dilation) {
    int ndim = input_size.size();
    std::vector<int64_t> output_size(ndim);
    for (int i = 0; i < ndim; i++){
        int size = (input_size[i] + 2 * padding[i] - dilation[i] * (kernel_size[i] - 1) - 1) / stride[i] + 1;
        if (kernel_size[i] == -1){
            output_size[i] = 1;
        } else
            output_size[i] = size;
    }
    return output_size;

}

void ops::dense(torch::Tensor &features, std::vector<int64_t> spatial_shape, torch::Tensor &indices, int batch_size){

    std::vector<int64_t> out_shape(5);
    out_shape[0] = batch_size;
    for(int i = 0; i < spatial_shape.size(); i++)
        out_shape[i+1] = spatial_shape[i];
    out_shape[4] = features.size(1);
//    auto res = torch::zeros({1, 2, 160, 132, 64}).toType(torch::kFloat32).to(torch::kCUDA, 0);
//
    auto res = ops::scatter_nd(indices, features, out_shape);
    int ndim = spatial_shape.size();
//    int * a = ndim;
    std::vector<int> trans_params(5);
    for (int i = 0; i < ndim + 2; ++i) {
        if (i == 0){
            trans_params[i] = i;
            continue;
        }
        if (i == 1){
            trans_params[i] = ndim + 1;
            continue;
        }
        trans_params[i] = i - 1;
    }

    features = res.permute({trans_params[0], trans_params[1], trans_params[2], trans_params[3], trans_params[4]}).contiguous();
}

torch::Tensor ops::scatter_nd(torch::Tensor &indice, torch::Tensor &features, std::vector<int64_t> &shape) {
    clock_t start = clock();
    auto ret = torch::zeros({shape[0],shape[1], shape[2], shape[3], shape[4]}).toType(torch::kFloat32).to(torch::kCUDA, 0);
    int ndim = indice.size(-1); // 4
    std::vector<int64_t> output_shape; //  [8328 64]
//    features.to(torch::kFloat32);
//    indice.to(torch::kInt16);
    output_shape.push_back(indice.size(0));
    output_shape.push_back(shape[indice.size(-1)]);

    indice = indice.view({-1, ndim}).to(torch::kLong);
    features = features.view({output_shape[0], output_shape[1]}); // [8328 64]
    // use xtensor to replace the index_put_ in libtorch;

    /**
    // xt::ret
    xt::xarray<float> ret = xt::zeros<float>({shape[0],shape[1], shape[2], shape[3], shape[4]});

    // feature : libtorch -> xt
    size_t fea_size = (size_t) features.size(0) * (size_t) features.size(1);
    std::vector<int> fea_shape = {(int) features.size(0), (int) features.size(1)};
    xt::xarray<float> fea_xt = xt::adapt(features.to(torch::kCPU).data_ptr<float>(), fea_size, xt::no_ownership(), fea_shape);

    // indices : libtorch -> xt
    size_t indice_size = (size_t) indice.size(0) * (size_t) indice.size(1);
    std::vector<int> indice_shape = {(int) indice.size(0), (int) indice.size(1)};
    xt::xarray<int> indice_xt = xt::adapt(indice.to(torch::kCPU).data_ptr<int64_t>(), indice_size, xt::no_ownership(), indice_shape);

    // calcaute
    for (int i = 0; i < indice_xt.shape(0); ++i) {
        xt::view(ret, indice_xt(i, 0), indice_xt(i, 1), indice_xt(i, 2), indice_xt(i, 3), xt::all()) = xt::view(fea_xt, i, xt::all());
    }

    // res : xt -> libtorch
    xt::xarray<int> ret_shape = xt::adapt(ret.shape());
    auto ress = torch::from_blob(ret.data(), {ret_shape(0), ret_shape(1), ret_shape(2), ret_shape(3), ret_shape(4)}, torch::kFloat32).to(torch::kCUDA);
    **/

     ret.index_put_({indice.index({"...", 0}),
                            indice.index({"...", 1}),
                            indice.index({"...", 2}),
                            indice.index({"...", 3}), }, features);
//    clock_t end = clock();
//
//    double t = (double) (end - start) / CLOCKS_PER_SEC; // s
//    std::cout<< "================ cost time : "<< t * 1000 << " ms!"<<std::endl;
    return ret;
}

