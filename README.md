# 点云预处理 Baseline

基于传统几何方法的工业点云预处理与工件提取 pipeline(参考东北大学《基于 3D 点云的平面角接焊缝特征提取与运动跟踪》)。

从工业 3D 相机采集的原始点云中去除背景与噪声,提取出工件点云,供后续焊缝特征提取使用。

## Pipeline

```
原始点云 (env / scene)
        ↓
Step 1  ROI 裁剪            roi_crop
        ↓
Step 2  体素下采样          voxel_downsample  (取距重心最近的原始点,保留细节)
        ↓
Step 3  统计离群点去除      statistical_filter (KD-tree 加速 kNN)
        ↓
Step 4  差异点云分割        diff_segmentation  (scene - env,KD-tree 最近邻)
        ↓
工件点云 workpiece_clean.ply
```

核心思想:`含工件场景点云 - 空环境点云 = 工件点云`。

## 目录结构

```
baseline/
├── include/                 # 头文件
│   ├── types.hpp            # 点云类型与 PipelineConfig
│   ├── ply_io.hpp           # PLY 读写(支持 ascii / binary_little_endian)
│   ├── kdtree.hpp           # KD-tree(最近邻 + kNN)
│   ├── roi_crop.hpp
│   ├── voxel_downsample.hpp
│   ├── statistical_filter.hpp
│   ├── diff_segmentation.hpp
│   └── pipeline.hpp         # 流程编排
├── src/                     # 各模块实现(一模块一文件)
├── main.cpp                 # 命令行入口,支持分步测试
└── CMakeLists.txt
scripts/
└── generate_test_data.py    # 生成测试用小点云
```

## 构建

需要 C++17 编译器。

```bash
cd baseline
clang++ -std=c++17 -O2 -Iinclude main.cpp src/*.cpp -o baseline
```

或使用 CMake:

```bash
cd baseline && mkdir -p build && cd build
cmake .. && cmake --build .
```

## 使用

支持逐步测试,`--step` 指定运行到哪一步:

```bash
# 仅 ROI 裁剪(默认)
./baseline --step roi   --env data/env.ply --scene data/scene.ply

# 运行到体素下采样 / 统计滤波 / 差分分割 / 完整流程
./baseline --step voxel
./baseline --step statistical
./baseline --step diff
./baseline --step all
```

每一步的中间结果会输出到 `output/baseline/`(如 `scene_roi.ply`、`scene_voxel.ply`、
`scene_filtered.ply`、`workpiece_clean.ply`),可用 CloudCompare / MeshLab 查看。

## 参数

默认参数在 `baseline/main.cpp` 的 `default_config()` 中(单位:mm):

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `roi` | 工作台包围盒 | 基于环境点云工作台层估计 |
| `voxel_size` | 1.0 | 体素边长 |
| `nb_neighbors` | 15 | 统计滤波 kNN 邻居数 |
| `std_ratio` | 4.0 | 统计滤波标准差倍数阈值 |
| `diff_threshold` | 3.0 | 差分最近邻距离阈值 |

## 测试数据

原始工业点云较大,未纳入版本控制。可用脚本生成小规模测试点云:

```bash
python3 scripts/generate_test_data.py
```
