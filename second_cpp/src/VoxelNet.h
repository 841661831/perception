//
// Created by root on 3/1/21.
//
#ifndef SPARSECONVOLUTIONPLUGIN_SECOND_H
#define SPARSECONVOLUTIONPLUGIN_SECOND_H
#include <iostream>
#include <string>
#include <torch/script.h>
#include "voxel_encoder.h"
#include "middle.h"
#include "rpn.h"
#include "target_assigner.h"
struct Preds_dict
{
    torch::Tensor box_preds;
    torch::Tensor cls_preds;
    torch::Tensor dir_cls_preds;
};

struct Pred_dict
{
    torch::Tensor box3d_lidar;
    torch::Tensor scores;
    torch::Tensor label_preds;
};

struct Example
{
    torch::Tensor voxels;
    torch::Tensor num_points;
    torch::Tensor coordinates;
    torch::Tensor num_voxels;
    torch::Tensor anchors;
} ;



class VoxelNet{
public:
    VoxelNet(std::vector<int64_t> output_shape,
            std::vector<float> voxel_size,  // voxel_generate && vfe
            std::vector<float> point_cloud_range, // voxel_generate && vfe
//            int64_t max_num_points,  // voxel_generate
//            int64_t max_voxels,     // voxel_generate
//            bool full_mean,   // voxel_generate
//            bool block_filtering,// voxel_generate
//            int64_t block_factor,// voxel_generate
//            int64_t block_size,// voxel_generate
//            float height_threshold,// voxel_generate
//            float height_high_threshold, // voxel_generate
            std::map<std::string, std::map<std::string, torch::Tensor>> &weights, // middle2
            //=======================================================
//             bool use_sigmoid_score,
             int nms_pre_max_sizes, //pre
             int nms_post_max_sizes,  //pre
             float nms_iou_thresholds, //pre
             std::vector<float> nms_score_thresholds,
             std::vector<struct Anchor_generators> anchor_generator,
             std::vector<std::vector<int>> feature_map_sizes,

             std::vector<int> feature_map_size = {1, 160, 132},
            //==========================================================
            int num_class = 2, //rpn
            int64_t num_input_features = 4, // vfe
            std::vector<int> vfe_num_filters = {32, 128}, // vfe
            bool with_distance = false, // vfe
            int middle_num_input_features = -1, // middle2
            int64_t rpn_num_input_features = -1, // rpn
            int64_t num_anchor_per_loc = 8,  // rpn
            std::vector<int64_t> rpn_layer_nums={3, 5, 5},  // rpn
            std::vector<int64_t> rpn_layer_strides={2, 2, 2},  // rpn
            std::vector<int64_t> rpn_num_filters={128, 128, 256},  // rpn
            std::vector<float> rpn_upsample_strides={1, 2, 4},  // rpn
            std::vector<int64_t> rpn_num_upsample_filters={256, 256, 256},  // rpn
            bool use_norm = true,// vfe && middle2 && rpn
            bool use_groupnorm=false,  // rpn
            int64_t num_groups=32,  // rpn
            int64_t box_code_size = 7,  // rpn
            bool use_direction_classifier=true, // rpn
            bool encode_background_as_zeros=true,// rpn
            int64_t num_direction_bins = 2, // rpn
            bool kd = false
    );


    std::map<std::string, torch::Tensor> forward(xt::xarray<float> &voxels, xt::xarray<int> &num_points,xt::xarray<int> &coors, int batch_size = 1);

    std::map<std::string, torch::Tensor> predict(torch::Tensor & example, std::map<std::string, torch::Tensor> & preds_dict);
//    Pred_dict & predict(Example & example, Preds_dict & preds_dict);

private:
    int _num_class;
    bool _use_rotate_nms; // pre
    bool _multiclass_nms; // pre
    std::vector<float> _nms_score_thresholds;  // pre
    int _nms_pre_max_sizes;  //pre
    int _nms_post_max_sizes;  //pre
    float _nms_iou_thresholds; //pre
    bool _use_sigmoid_score; // pre
    bool _encode_background_as_zeros;  // pre
    bool _use_direction_classifier;  //pre
    bool _box_coder;
    TargetAssigner _target_assigner;  // TODO : add in the future
    float _dir_offset; // pre
    std::vector<float> _post_center_range; // pre
    bool _nms_class_agnostic; // pre
    int _num_direction_bins; //pre
    float _dir_limit_offset; // pre
//    VoxelGeneratorV2 _voxel_generator;
    SimpleVoxelRadius _voxel_feature_extractor;
    SpMiddleFHD _middle_feature_extractor;
    RPNV2 _rpn;
    std::map<std::string, torch::Tensor> _weights_middle;
    std::map<std::string, torch::Tensor> _weights_rpn;

    torch::Tensor _anchors;
};


// ======================  predict function ==============================
torch::Tensor rotate_nms( const torch::Tensor & boxes_for_nms,
                          const torch::Tensor & top_scores,
                          int pre_max_sizes,
                          int post_max_sizes,
                          float iou_thresholds);

torch::Tensor rotate_nms_cc(const torch::Tensor & dets_np, float thresh);

xt::xarray<float> iou_jit( const torch::Tensor & boxes,
                           const torch::Tensor & query_boxes,
                           float eps=1.0);

torch::Tensor corner_to_standup_nd(const torch::Tensor & dets_corners);

torch::Tensor center_to_corner_box2d(torch::Tensor & centers,
                                     torch::Tensor & dims,
                                     torch::Tensor & angles,
                                     float origin=0.5);

torch::Tensor rotation_2d(const torch::Tensor & corners,
                          const torch::Tensor & angles);

torch::Tensor corners_nd(torch::Tensor & dims, float origin=0.5 );

torch::Tensor decode_torch(const torch::Tensor& box_encodings,
                           const torch::Tensor& anchors);

torch::Tensor second_box_decode(const torch::Tensor& box_encodings,
                                const torch::Tensor& anchors,
                                bool encode_angle_to_vector,
                                bool smooth_dim);

torch::Tensor rotate_non_max_suppression_cpu(const torch::Tensor & t_box_corners,
                                             const torch::Tensor & t_order,
                                             const xt::xarray<float> & t_standup_iou,
                                             float thresh);

template <typename DType>
xt::xarray<DType> tensorToxTensor(const torch::Tensor & boxes);
// =============================================================================

#endif //SPARSECONVOLUTIONPLUGIN_SECOND_H