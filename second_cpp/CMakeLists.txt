cmake_minimum_required(VERSION 3.10)
project(SECOND_detector VERSION 1.0)


# torch
set(Torch_DIR "/usr/local/lib/python3.6/dist-packages/torch/share/cmake/Torch")
find_package(Torch REQUIRED)
set(ALL_LIBS ${TORCH_LIBRARIES})

# xtenror
find_package(xtensor REQUIRED)

# pcl
#find_package(PCL 1.2 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

# Open3d
#find_package(Eigen3 REQUIRED)
#find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
#list(APPEND Open3D_LIBRARIES dl)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")
#add_definitions(-DUNIX)
#add_compile_options(-Wno-deprecated-declarations)
#add_compile_options(-Wno-unused-result)
#add_definitions(-O3)
#link_directories(${Open3D_LIBRARY_DIRS})

# cuda & cudnn
find_package(CUDA REQUIRED)
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
get_filename_component(CUDA_TOOLKIT_ROOT_DIR "${CUDA_TOOLKIT_ROOT_DIR}" DIRECTORY)
get_filename_component(CUDA_TOOLKIT_ROOT_DIR "${CUDA_TOOLKIT_ROOT_DIR}" DIRECTORY)
set(CUDA_LIB_PATH_HINTS "${CUDA_TOOLKIT_ROOT_DIR}/lib64")
find_library(CUDA_CUDART NAMES cudart HINTS ${CUDA_LIB_PATH_HINTS})
find_library(CUDA_CUBLAS NAMES cublas HINTS ${CUDA_LIB_PATH_HINTS})
torch_cuda_get_nvcc_gencode_flag(NVCC_FLAGS_EXTRA)
string (REPLACE ";" " " NVCC_FLAGS_EXTRA_STR "${NVCC_FLAGS_EXTRA}")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} ${NVCC_FLAGS_EXTRA_STR}")
include_directories(/usr/local/include)
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)



add_executable(SECOND_detector main.cpp inference.cpp inference.h)
include_directories("src/")
add_subdirectory(src)


set_target_properties(SECOND_detector PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

target_include_directories(SECOND_detector PRIVATE ${EIGEN3_INCLUDE_DIR} ${Open3D_INCLUDE_DIRS} ${xtensor_INCLUDE_DIRS})

target_link_libraries(SECOND_detector spconv_cpp ${Open3D_LIBRARIES} -lpthread -lm
        /data/second_cpp/src/libspconv.so
        /data/second_cpp/src/libcuhash.so
        ${TORCH_LIBRARIES}
        /usr/lib/x86_64-linux-gnu/libz.so
        xtensor
        )

# open3d
#target_include_directories(SECOND_detector PUBLIC ${EIGEN3_INCLUDE_DIR} ${Open3D_INCLUDE_DIRS} )
#target_link_libraries(SECOND_detector ${Open3D_LIBRARIES} -lpthread -lm)
