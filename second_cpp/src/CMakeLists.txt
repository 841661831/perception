aux_source_directory(. LIB_SRCS)
add_library (spconv_cpp ${LIB_SRCS} ops.cpp ops.h
        SparseConvolution.cpp SparseConvolution.h
        middle.cpp middle.h
        cnpy.cpp cnpy.h voxel_encoder.cpp voxel_encoder.h
        VoxelNet.cpp VoxelNet.h target_assigner.cpp target_assigner.h)
target_include_directories(spconv_cpp PRIVATE ${TORCH_INCLUDE_DIRS})
target_link_libraries(spconv_cpp "${TORCH_LIBRARIES}")
