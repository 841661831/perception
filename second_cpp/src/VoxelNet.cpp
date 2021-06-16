//
// Created by chenxiangyang on 3/2/21.
//
#include <iostream>
#include "VoxelNet.h"
#include <vector>
#include <string>
#include <torch/script.h>
#include "voxel_encoder.h"
#include "middle.h"
#include <ctime>
#include <boost/geometry.hpp>
#include "../inference.h"
//Pred_dict prediction_dict;
std::map<std::string, torch::Tensor> prediction_dict;

VoxelNet::VoxelNet(std::vector<int64_t> output_shape, // 1
                   std::vector<float> voxel_size,  // voxel_generate && vfe
                   std::vector<float> point_cloud_range,  // voxel_generate && vfe
//                   int64_t max_num_points,  // voxel_generate
//                   int64_t max_voxels,      // voxel_generate
//                   bool full_mean,    // voxel_generate
//                   bool block_filtering, // voxel_generate
//                   int64_t block_factor,   // voxel_generate
//                   int64_t block_size,   // voxel_generate
//                   float height_threshold,  // voxel_generate
//                   float height_high_threshold,   // voxel_generate
                   std::map<std::string, std::map<std::string, torch::Tensor>> &weights, // middle2
//                   bool use_sigmoid_score,
                   int nms_pre_max_sizes, //pre
                   int nms_post_max_sizes,  //pre
                   float nms_iou_thresholds, //pre
                   std::vector<float> nms_score_thresholds,

                   std::vector<struct Anchor_generators> anchor_generator,  // target
                   std::vector<std::vector<int>> feature_map_sizes,  //target
                   std::vector<int> feature_map_size, //target
                   int num_class,   //rpn
                   int64_t num_input_features, // vfe
                   std::vector<int> vfe_num_filters,  // vfe
                   bool with_distance, // vfe
                   int32_t middle_num_input_features, // middle2
                   int64_t rpn_num_input_features,  // rpn
                   int64_t num_anchor_per_loc,  // rpn
                   std::vector<int64_t> rpn_layer_nums,   // rpn
                   std::vector<int64_t> rpn_layer_strides,  // rpn
                   std::vector<int64_t> rpn_num_filters,  // rpn
                   std::vector<float> rpn_upsample_strides,  // rpn
                   std::vector<int64_t> rpn_num_upsample_filters,   // rpn

                   bool use_norm,  // vfe && middle2 && rpn
                   bool use_groupnorm,  // rpn
                   int64_t num_groups,   // rpn
                   int64_t box_code_size, // rpn  TODO change it to target_assigner
                   bool use_direction_classifier, // rpn
                   bool encode_background_as_zeros,  // rpn
                   int64_t num_direction_bins,  // rpn
                   bool kd
                   ):
                _num_class(num_class),
                _box_coder(box_code_size),
                _num_direction_bins(num_direction_bins),
                _encode_background_as_zeros(encode_background_as_zeros),
                _use_direction_classifier(use_direction_classifier),
                _weights_middle(weights["middle2"]),
                _weights_rpn(weights["rpn"]),
//                _use_sigmoid_score(use_sigmoid_score),
                _nms_score_thresholds(nms_score_thresholds),
                _nms_pre_max_sizes(nms_pre_max_sizes),
                _nms_post_max_sizes(nms_post_max_sizes),
                _nms_iou_thresholds(nms_iou_thresholds)
        {
//    _voxel_generator = VoxelGeneratorV2(voxel_size, point_cloud_range, max_num_points, max_voxels, full_mean, block_filtering,
//                                        block_factor, block_size, height_threshold, height_high_threshold);

//    _voxel_feature_extractor = SimpleVoxelRadius(num_input_features, use_norm, vfe_num_filters, with_distance, _voxel_generator._voxel_size, point_cloud_range);


    _target_assigner = TargetAssigner(anchor_generator,feature_map_sizes);
    _anchors = _target_assigner.generate_anchors(feature_map_size);

    _middle_feature_extractor = SpMiddleFHD(output_shape, use_norm, middle_num_input_features, _weights_middle, kd);

    _rpn = RPNV2(use_norm, num_class, rpn_layer_nums, rpn_layer_strides, rpn_num_filters, rpn_upsample_strides, rpn_num_upsample_filters,
                 rpn_num_input_features, num_anchor_per_loc, encode_background_as_zeros, use_direction_classifier, use_groupnorm,
                 num_groups, box_code_size, _num_direction_bins, false);
    _rpn.load_weight(_weights_rpn, true, false);



}





template <typename DType>
xt::xarray<DType> tensorToxTensor(const torch::Tensor & boxes)
{
    int n = boxes.dim();
    if (n == 1)
    {
        size_t size = (size_t) boxes.size(0) ;
        std::vector<DType> tensorshape = {DType (boxes.size(0))};
        xt::xarray<DType> out = xt::adapt(boxes.data_ptr<DType>(), size, xt::no_ownership(), tensorshape);
        return out;
    }else if (n == 2)
    {
        size_t size = (size_t) boxes.size(0) * (size_t) boxes.size(1);
        std::vector<DType> tensorshape = {DType (boxes.size(0)),DType (boxes.size(1))};
        xt::xarray<DType> out = xt::adapt(boxes.data_ptr<DType>(), size, xt::no_ownership(), tensorshape);
        return out;
    }else if(n == 3)
    {
        size_t size = (size_t) boxes.size(0) * (size_t) boxes.size(1)* (size_t) boxes.size(2);
        std::vector<DType> tensorshape = {DType (boxes.size(0)),DType (boxes.size(1)),DType (boxes.size(2))};
        xt::xarray<DType> out = xt::adapt(boxes.data_ptr<DType>(), size, xt::no_ownership(), tensorshape);
        return out;
    }
    else
    {
        std::cout << "wrong dim" << std::endl;
    }
    return -1;

}

xt::xarray<float> iou_jit( const torch::Tensor & boxes,
                           const torch::Tensor & query_boxes,
                           float eps){

    xt::xarray<float> xt_boxes = tensorToxTensor<float> (boxes);
    xt::xarray<float> xt_query_boxes = tensorToxTensor<float> (query_boxes);
    int N = xt_boxes.shape(0);
    int K = xt_query_boxes.shape(0);
    xt::xarray<float> overlaps = xt::zeros<float>({N,K});

    float box_area;
    float iw;
    float ih;
    float ua;

    for (int k = 0; k < K; ++k)
    {
        box_area = ((xt_query_boxes(k, 2) - xt_query_boxes(k, 0) + eps) * (xt_query_boxes(k, 3) - xt_query_boxes(k ,1) + eps));
        for (int n = 0; n < N; ++n)
        {
            iw = std::min(xt_boxes(n, 2), xt_query_boxes(k, 2)) - std::max(xt_boxes(n, 0), xt_query_boxes(k, 0) + eps);
            if (iw > 0)
            {
                ih = std::min(xt_boxes(n, 3), xt_query_boxes(k, 3)) - std::max(xt_boxes(n, 1), xt_query_boxes(k, 1) + eps);
                if (ih > 0)
                {
                    ua = ((xt_boxes(n, 2) - xt_boxes(n, 0) + eps) * (xt_boxes(n, 3) - xt_boxes(n, 1) + eps) + box_area - iw * ih);
                    overlaps(n, k) = iw * ih / ua;
                }
            }
        }

    }

    return overlaps;
}

torch::Tensor corner_to_standup_nd(const torch::Tensor & dets_corners){

    assert(dets_corners.sizes().size() == 3);
    std::tuple<torch::Tensor,torch::Tensor> max_standup_boxes;
    std::tuple<torch::Tensor,torch::Tensor> min_standup_boxes;
    torch::Tensor x_standup_boxes;
    torch::Tensor n_standup_boxes;
    std::vector<torch::Tensor> standup_boxes;

    min_standup_boxes = torch::min({dets_corners},1);
    max_standup_boxes = torch::max({dets_corners},1);

    n_standup_boxes = std::get<0>(min_standup_boxes);
    x_standup_boxes = std::get<0>(max_standup_boxes);

    standup_boxes.push_back(n_standup_boxes);
    standup_boxes.push_back(x_standup_boxes);

    torch::Tensor res4 = torch::cat({standup_boxes}, -1);

    return res4;
}


torch::Tensor rotation_2d(const torch::Tensor & corners, const torch::Tensor & angles){
    torch::Tensor rot_sin = torch::sin(angles);
    torch::Tensor rot_cos = torch::cos(angles);
    torch::Tensor a = torch::cat({rot_cos,-rot_sin},1).t();
    torch::Tensor b = torch::cat({rot_sin,rot_cos},1).t();
    torch::Tensor rot_mat_T = torch::stack({a, b},0);
    torch::Tensor res3 = torch::einsum("aij,jka->aik",{corners , rot_mat_T});
    return res3;
}

torch::Tensor corners_nd(torch::Tensor & dims, float origin){

    int ndim = int(dims.size(1));
    torch::Tensor corners_norm;
    if(ndim == 2)
    {
        corners_norm = torch::zeros({4,2});
        corners_norm[1][1] = 1;
        corners_norm[2][0] = 1;
        corners_norm[3][0] = 1;
        corners_norm[2][1] = 1;
        corners_norm.to(torch::kFloat32);
    }
    else if(ndim == 3)
    {
        corners_norm = torch::zeros({8,3});
        corners_norm[4][0] = 1;
        corners_norm[5][0] = 1;
        corners_norm[6][0] = 1;
        corners_norm[7][0] = 1;
        corners_norm[2][1] = 1;
        corners_norm[3][1] = 1;
        corners_norm[6][1] = 1;
        corners_norm[7][1] = 1;
        corners_norm[1][2] = 1;
        corners_norm[2][2] = 1;
        corners_norm[5][2] = 1;
        corners_norm[6][2] = 1;
        corners_norm.to(torch::kFloat32);
    }
    corners_norm = corners_norm - origin;
    torch::Tensor corners;
    int x = pow(2,ndim);
    corners = dims.reshape({-1,1,ndim})*corners_norm.reshape({1,x,ndim});
    return corners;
}

torch::Tensor center_to_corner_box2d(torch::Tensor & centers,
                                     torch::Tensor & dims,
                                     torch::Tensor & angles,
                                     float origin){
    torch::Tensor corners = corners_nd(dims, origin);  // corners: [N, 4, 2]
    if (angles.size(0) != 0)
    {
        corners = rotation_2d(corners, angles);
    }
    corners += centers.reshape({-1, 1, 2});
    return corners;
}



torch::Tensor second_box_decode(const torch::Tensor& box_encodings,
                                const torch::Tensor& anchors,
                                bool encode_angle_to_vector,
                                bool smooth_dim)
{


    int box_ndim = anchors.sizes()[2];
    //boxes ([N, 7] Tensor): normal boxes: x, y, z, w, l, h, r
    //note: for encode_angle_to_vector is true: r==rx ry ==ry
    // for encode_angle_to_vector is false: r == r, ry not use
    enum anchor_type{x__ = 0, y__, z__, w__, l__, h__, r__, ry__};

    std::vector<torch::Tensor> vec_anchors = torch::split(anchors, 1, -1);
    std::vector<torch::Tensor> vec_box_encodings = torch::split(box_encodings, 1, -1);

    torch::Tensor diagonal = torch::sqrt(vec_anchors.at(l__) * vec_anchors.at(l__)
                                         + vec_anchors.at(w__) * vec_anchors.at(w__));

    torch::Tensor xg = vec_box_encodings.at(x__) * diagonal + vec_anchors.at(x__);
    torch::Tensor yg = vec_box_encodings.at(y__) * diagonal + vec_anchors.at(y__);
    torch::Tensor zg = vec_box_encodings.at(z__) * vec_anchors.at(h__) + vec_anchors.at(z__);

    torch::Tensor lg;
    torch::Tensor wg;
    torch::Tensor hg;
    if (smooth_dim)
    {
        lg = (vec_box_encodings.at(l__) + 1) * vec_anchors.at(l__);
        wg = (vec_box_encodings.at(w__) + 1) * vec_anchors.at(w__);
        hg = (vec_box_encodings.at(h__) + 1) * vec_anchors.at(h__);
    }
    else
    {
        lg = torch::exp(vec_box_encodings.at(l__)) * vec_anchors.at(l__);
        wg = torch::exp(vec_box_encodings.at(w__)) * vec_anchors.at(w__);
        hg = torch::exp(vec_box_encodings.at(h__)) * vec_anchors.at(h__);
    }

    torch::Tensor rg;
    if (encode_angle_to_vector)
    {
        torch::Tensor rgx = vec_box_encodings.at(r__) + torch::cos(vec_anchors.at(r__));
        torch::Tensor rgy = vec_box_encodings.at(ry__) + torch::sin(vec_anchors.at(r__));
        rg = torch::atan2(rgy, rgx);
    }
    else
    {
        rg = vec_box_encodings.at(r__) + vec_anchors.at(r__);
    }

    //get cgs
    std::vector<torch::Tensor> cgs;
    if (box_ndim > 7)
    {
        std::vector<torch::Tensor>::iterator it_anchors = vec_anchors.begin();
        std::vector<torch::Tensor>::iterator it_box_encoding = vec_box_encodings.begin();
        int minimum_size = std::min(vec_anchors.size(), vec_box_encodings.size());

        it_anchors += r__ + 1;
        if (encode_angle_to_vector)
        {
            it_box_encoding += ry__ + 1;
        }
        else
        {
            it_box_encoding += r__ + 1;
        }
        while (it_anchors < vec_anchors.end() && it_box_encoding < vec_box_encodings.end())
        {
            cgs.emplace_back(*it_anchors + *it_box_encoding);
            it_box_encoding++;
            it_anchors++;
        }
    }
    else
    {
        cgs.clear();
    }

    std::vector<torch::Tensor> temp;
    temp.push_back(std::move(xg));
    temp.push_back(std::move(yg));
    temp.push_back(std::move(zg));
    temp.push_back(std::move(wg));
    temp.push_back(std::move(lg));
    temp.push_back(std::move(hg));
    temp.push_back(std::move(rg));

    std::vector<torch::Tensor>::iterator it;
    for (it = cgs.begin(); it < cgs.end(); it++)
    {
        temp.push_back(std::move(*it));
    }

    torch::Tensor res = torch::cat(temp, -1);
    return res;
}

torch::Tensor decode_torch(const torch::Tensor& box_encodings, const torch::Tensor& anchors){
    auto res = second_box_decode(box_encodings, anchors, false, false);
    return res;
}



torch::Tensor rotate_non_max_suppression_cpu( const torch::Tensor & t_box_corners,
                                              const torch::Tensor & t_order,
                                              const xt::xarray<float> & t_standup_iou,
                                             float thresh) {

    xt::xarray<float> box_corners = tensorToxTensor<float>(t_box_corners);
    xt::xarray<int> order1 = tensorToxTensor<int>(t_order.cpu());
    order1 = order1.reshape({-1});



    int ndets = t_box_corners.size(0);
    xt::xarray<float> box_corners_r = box_corners;
    xt::xarray<float> order_r = order1;
    xt::xarray<int> suppressed = xt::zeros<int>({ndets});
    xt::xarray<int> suppressed_rw = suppressed;
    xt::xarray<float> standup_iou_r = t_standup_iou;
    std::vector<int> keep;
    int i, j;
    namespace bg = boost::geometry;
    typedef bg::model::point<float, 2, bg::cs::cartesian> point_t;
    typedef bg::model::polygon<point_t> polygon_t;
    polygon_t poly, qpoly;
    std::vector<polygon_t> poly_inter, poly_union;
    float inter_area, union_area, overlap;

    for (int _i = 0; _i < ndets; ++_i) {
        i = order_r(_i);
        if (suppressed_rw(i) == 1)
            continue;
        keep.push_back(i);
        for (int _j = _i + 1; _j < ndets; ++_j) {
            j = order_r(_j);
            if (suppressed_rw(j) == 1)
                continue;
            if (standup_iou_r(i, j) <= 0.0)
                continue;
            try {
                bg::append(poly,
                           point_t(box_corners_r(i, 0, 0), box_corners_r(i, 0, 1)));
                bg::append(poly,
                           point_t(box_corners_r(i, 1, 0), box_corners_r(i, 1, 1)));
                bg::append(poly,
                           point_t(box_corners_r(i, 2, 0), box_corners_r(i, 2, 1)));
                bg::append(poly,
                           point_t(box_corners_r(i, 3, 0), box_corners_r(i, 3, 1)));
                bg::append(poly,
                           point_t(box_corners_r(i, 0, 0), box_corners_r(i, 0, 1)));
                bg::append(qpoly,
                           point_t(box_corners_r(j, 0, 0), box_corners_r(j, 0, 1)));
                bg::append(qpoly,
                           point_t(box_corners_r(j, 1, 0), box_corners_r(j, 1, 1)));
                bg::append(qpoly,
                           point_t(box_corners_r(j, 2, 0), box_corners_r(j, 2, 1)));
                bg::append(qpoly,
                           point_t(box_corners_r(j, 3, 0), box_corners_r(j, 3, 1)));
                bg::append(qpoly,
                           point_t(box_corners_r(j, 0, 0), box_corners_r(j, 0, 1)));

                bg::intersection(poly, qpoly, poly_inter);  //slow  jiaoji
            }catch (const std::exception &e) {
                std::cout << "box i corners:" << std::endl;
                for (int k = 0; k < 4; ++k) {
                    std::cout << box_corners_r(i, k, 0) << " " << box_corners_r(i, k, 1)
                              << std::endl;
                }
                std::cout << "box j corners:" << std::endl;
                for (int k = 0; k < 4; ++k) {
                    std::cout << box_corners_r(j, k, 0) << " " << box_corners_r(j, k, 1)
                              << std::endl;
                }
                continue;
            }
            if (!poly_inter.empty()) {
                inter_area = bg::area(poly_inter.front());
                bg::union_(poly, qpoly, poly_union);   //slow  bingji
                if (!poly_union.empty()) { // ignore invalid box
                    union_area = bg::area(poly_union.front());
                    overlap = inter_area / union_area;
                    if (overlap >= thresh)
                        suppressed_rw(j) = 1;
                    poly_union.clear();
                }
            }
            poly.clear();
            qpoly.clear();
            poly_inter.clear();
        }
    }
    int t = keep.size();
    torch::Tensor result = torch::tensor(keep).to(torch::kInt32).to(torch::kCUDA);
//    for (int k = 0; j < keep.size(); ++k) {
//        result[k] = keep[k];
//    }
//    std::cout << "keeps= \n"<<keep << std::endl;
//    std::cout << "result= \n"<<result << std::endl;
    return result;
}





torch::Tensor rotate_nms_cc(const torch::Tensor & dets_np, float thresh){


    torch::Tensor c_scores = dets_np.index({"...", torch::tensor({5})}).to(torch::kCUDA);
    torch::Tensor c_order = c_scores.argsort(0,true).to(torch::kInt32).to(torch::kCPU);
    torch::Tensor centers =  dets_np.index({"...", torch::tensor({0,1})});
    torch::Tensor dims =  dets_np.index({"...", torch::tensor({2,3})});
    torch::Tensor angles = dets_np.index({"...", torch::tensor({4})});


    auto t1 = std::chrono::steady_clock::now();

    torch::Tensor dets_corners = center_to_corner_box2d(centers, dims, angles );

    auto t2 = std::chrono::steady_clock::now();
    double dr = std::chrono::duration<double, std::milli>(t2 - t1).count();
//    printf("center_to_corner_box2d cost  %f\n",dr);

    auto t3 = std::chrono::steady_clock::now();
    torch::Tensor dets_standup = corner_to_standup_nd(dets_corners);
    auto t4 = std::chrono::steady_clock::now();
    double dq = std::chrono::duration<double, std::milli>(t4 - t3).count();
//    printf("corner_to_standup_nd cost  %f\n",dq);

    auto t5 = std::chrono::steady_clock::now();
    xt::xarray<float> standup_iou = iou_jit(dets_standup, dets_standup, 0.0);
    auto t6 = std::chrono::steady_clock::now();
    double dw = std::chrono::duration<double, std::milli>(t6 - t5).count();
//    printf("iou_jit cost  %f\n",dw);

    auto t7 = std::chrono::steady_clock::now();

    torch::Tensor res5 = rotate_non_max_suppression_cpu(dets_corners,c_order,standup_iou, thresh);
//    std::cout << "res5 = \n"<< res5 << std::endl;
    auto t8 = std::chrono::steady_clock::now();
    double di = std::chrono::duration<double, std::milli>(t8 - t7).count();
//    printf("rotate_non_max_suppression_cpu cost  %f\n",di);

    return res5;
}

torch::Tensor rotate_nms( const torch::Tensor & boxes_for_nms,
                          const torch::Tensor & top_scores,
                          int pre_max_sizes,
                          int post_max_sizes,
                          float iou_thresholds)
{

    torch::Tensor rbboxes = boxes_for_nms;
    torch::Tensor scores;
    std::tuple<torch::Tensor,torch::Tensor> topk;
    int num_keeped_scores = top_scores.size(0);
    pre_max_sizes = std::min(num_keeped_scores, pre_max_sizes);
    topk = torch::topk(top_scores, pre_max_sizes);
    scores = std::get<0>(topk);
    torch::Tensor indices = std::get<1>(topk);
    rbboxes = rbboxes.index({indices.to(torch::kLong)});
    torch::Tensor dets;
    dets = torch::cat({rbboxes, scores.unsqueeze(-1)},1);

    torch::Tensor dets_np =  dets.data().cpu();



    torch::Tensor ret = rotate_nms_cc(dets_np, iou_thresholds);


    torch::Tensor res2 = indices.index({ret.to(torch::kLong)}).to(torch::kInt32);

    return res2;

}

std::map<std::string, torch::Tensor> VoxelNet::predict(torch::Tensor & example, std::map<std::string, torch::Tensor> & preds_dict)
{


    torch::Tensor total_scores;
    torch::Tensor batch_anchors;
    torch::Tensor batch_box_preds;
    torch::Tensor batch_cls_preds;
    torch::Tensor batch_dir_preds;

    int batch_size = example.size(0);
    batch_anchors = example.view({batch_size,-1,example.size(-1)});
    batch_box_preds = preds_dict["box_preds"];
    batch_cls_preds = preds_dict["cls_preds"];
//    batch_box_preds = torch::where(torch::isnan(batch_box_preds), torch::full_like(batch_box_preds, 0), batch_box_preds);
//    batch_box_preds = torch::where(torch::isinf(batch_box_preds), torch::full_like(batch_box_preds, 0), batch_box_preds);
//    batch_cls_preds = torch::where(torch::isnan(batch_cls_preds), torch::full_like(batch_cls_preds, 0), batch_cls_preds);
//    batch_cls_preds = torch::where(torch::isinf(batch_cls_preds), torch::full_like(batch_cls_preds, 0), batch_cls_preds);

    batch_box_preds = batch_box_preds.view({batch_size,-1, 7});  //  7  =  m_box_coder.code_size


    int num_class_with_pg = _num_class;
    if (!_encode_background_as_zeros)
    {
        num_class_with_pg = _num_class + 1;
    }
    batch_cls_preds = batch_cls_preds.view({batch_size,-1,num_class_with_pg});

    batch_box_preds = decode_torch(batch_box_preds,batch_anchors);

    if (_use_direction_classifier)
    {
        batch_dir_preds = preds_dict["dir_cls_preds"];
//        batch_dir_preds = torch::where(torch::isnan(batch_dir_preds), torch::full_like(batch_dir_preds, 0), batch_dir_preds);
//        batch_dir_preds = torch::where(torch::isinf(batch_dir_preds), torch::full_like(batch_dir_preds, 0), batch_dir_preds);
        batch_dir_preds = batch_dir_preds.view({batch_size,-1,_num_direction_bins});
    }




    torch::Tensor post_center_range = torch::tensor(
            {0.0, -40.0, -2.200000047683716, 70.4000015258789, 40.0, 0.800000011920929})
                               .to(batch_box_preds.dtype())
                               .to(batch_box_preds.device());

    torch::Tensor box_preds = batch_box_preds[0];
    torch::Tensor cls_preds = batch_cls_preds[0];
    torch::Tensor dir_preds = batch_dir_preds[0];
//    std::cout<<batch_box_preds[0];


    std::tuple<torch::Tensor,torch::Tensor> tmp_dir_labels;
    torch::Tensor dir_labels ;
    torch::Tensor boxes_for_nms;
    torch::Tensor top_scores_keep;
    torch::Tensor selected;
    torch::Tensor selected_boxes;
    torch::Tensor selected_dir_labels;
    torch::Tensor selected_labels;
    torch::Tensor selected_scores;
    torch::Tensor scores;
    torch::Tensor label_preds;

    if (_use_direction_classifier)
    {
        tmp_dir_labels = torch::max(dir_preds,-1);
        dir_labels = std::get<1>(tmp_dir_labels);
    }
    if (_encode_background_as_zeros)
    {
//        assert(_use_sigmoid_score == true);
        total_scores = torch::sigmoid(cls_preds);
    }

    int feature_map_size_prod = 21120;       //    feature_map_size_prod = batch_box_preds.size(1) / _target_assigner.num_anchors_per_location;
    std::tuple<torch::Tensor,torch::Tensor> top = torch::max(total_scores,-1);     //   std::get<0>(top):top_scores      std::get<1>(top):top_labels
    torch::Tensor top_scores = std::get<0>(top);
    torch::Tensor top_labels = std::get<1>(top);

    if (_nms_score_thresholds[0] > 0.0)
    {
        top_scores_keep = top_scores >= _nms_score_thresholds[0];
        top_scores = top_scores.masked_select(top_scores_keep);
    }
    if (top_scores.size(0) != 0)
    {
        if (_nms_score_thresholds[0] > 0.0 )
        {

            box_preds = box_preds.index({top_scores_keep.to(torch::kBool)});
            if (_use_direction_classifier)
            {
                dir_labels = dir_labels.index({top_scores_keep.to(torch::kBool)});
            }
            top_labels = top_labels.index({top_scores_keep.to(torch::kBool)});
        }

        boxes_for_nms = box_preds.index({"...", torch::tensor({0, 1, 3, 4, 6})});

        auto t1 = std::chrono::steady_clock::now();

        selected = rotate_nms(boxes_for_nms, top_scores, _nms_pre_max_sizes, _nms_post_max_sizes, _nms_iou_thresholds);

        auto t2 = std::chrono::steady_clock::now();
        double dr = std::chrono::duration<double, std::milli>(t2 - t1).count();
//        printf("rotate_nms cost  %f\n",dr);
    }

//    std::cout<<box_preds;
    selected_boxes = box_preds.index({selected.to(torch::kLong)});

    if (_use_direction_classifier)
    {
        selected_dir_labels = dir_labels.index({selected.to(torch::kLong)});
    }
    selected_labels = top_labels.index({selected.to(torch::kLong)});
    selected_scores = top_scores.index({selected.to(torch::kLong)});


    // finally generate predictions.
    if (selected_boxes.size(0) != 0)
    {
        box_preds = selected_boxes;
        scores = selected_scores;
        label_preds = selected_labels;

        if (_use_direction_classifier)
        {
            dir_labels = selected_dir_labels;

        }


        torch::Tensor final_box_preds = box_preds;
        torch::Tensor final_scores = scores;
        torch::Tensor final_labels = label_preds;




        torch::Tensor mask = (final_box_preds.index({"...", torch::tensor({0,1,2})})
                >= post_center_range.slice(0, 0, 3)).to(torch::kBool).all(1);
        mask &= (final_box_preds.index({"...", torch::tensor({0,1,2})})
                <= post_center_range.slice(0, 3, final_box_preds.size(1) - 1)).to(torch::kBool).all(1);


        prediction_dict["box3d_lidar"] = final_box_preds.index({mask.to(torch::kBool)});
        prediction_dict["scores"] = final_scores.index({mask.to(torch::kBool)});
        prediction_dict["label_preds"] = label_preds.index({mask.to(torch::kBool)});

    }

    return prediction_dict;

}

std::map<std::string, torch::Tensor> VoxelNet::forward(xt::xarray<float> &voxels, xt::xarray<int> &num_points,xt::xarray<int> &coors, int batch_size) {

    xt::xarray<int> coor_shape = xt::adapt(coors.shape());
    torch::Tensor coors_t = torch::from_blob(coors.data(), {coor_shape(0), coor_shape(1)}, torch::kInt32).to(torch::kCUDA);

    auto t1 = std::chrono::steady_clock::now();
    auto voxel_feature = _voxel_feature_extractor.forward_xt(voxels, num_points);

    auto t2 = std::chrono::steady_clock::now();

    std::cout<<voxel_feature.sizes()<<"\n";
    auto spatial_features = _middle_feature_extractor.forward(voxel_feature, coors_t, batch_size);
    auto t3 = std::chrono::steady_clock::now();

    auto preds_dict = _rpn.forward(spatial_features);
    auto t4 = std::chrono::steady_clock::now();
//    std::cout<<preds_dict["box_preds"].index({0, 0, 0})<<"\n";
//    std::cout<<preds_dict["cls_preds"]<<"\n";
//    std::cout<<preds_dict["dir_cls_preds"]<<"\n";


//    auto anchor = npy2tensor("/data/second_cpp2/weights2/weights/anchor.npy");
    auto res = predict(_anchors, preds_dict);
//    std::cout<<res["box3d_lidar"] << "\n";
//    std::cout<<res["scores"] << "\n";
//    std::cout<<res["label_preds"] << "\n";


    auto t5 = std::chrono::steady_clock::now();


    double vfe = std::chrono::duration<double, std::milli>(t2 - t1).count();
    double mid = std::chrono::duration<double, std::milli>(t3 - t2).count();
    double rpn = std::chrono::duration<double, std::milli>(t4 - t3).count();
    double pre = std::chrono::duration<double, std::milli>(t5 - t4).count();
    printf("vfe cost %4f ms!\n", vfe);
    printf("mid cost %4f ms!\n", mid);
    printf("rpn cost %4f ms!\n", rpn);
    printf("pre cost %4f ms!\n", pre);
    return res;
}
