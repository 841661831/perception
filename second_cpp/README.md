### 20210407

当前版本增加了知识蒸馏.

主要改动:

1. 在创建middlenet部分,减去了6层稀疏卷积. 
2. 在voxelnet中增加了kd参数
3. 在voxelnet中增加了target_assigner部分


weights文件可以去文件网盘中下载.
1. weights2 : 正常的kitti权重
2. weights_kd : 知识蒸馏后的权重
3. weights_32l : 32line数据权重.