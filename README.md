# 点云预处理 Baseline

基于传统几何方法的工业点云预处理与工件提取 pipeline(参考东北大学《基于 3D 点云的平面角接焊缝特征提取与运动跟踪》)。

从工业 3D 相机采集的原始点云中去除背景与噪声,提取出工件点云,供后续焊缝特征提取使用。

## Pipeline

```
原始点云 (env / scene)
        ↓
Step 1  ROI 裁剪            roi_crop           (env/scene 同一包围盒)
        ↓
Step 2  体素下采样          voxel_downsample   (取距重心最近的原始点,保留细节)
        ↓
Step 3  差异点云分割        diff_segmentation  (scene - env,KD-tree 最近邻)
        ↓
Step 4  统计离群点去除      statistical_filter (仅作用于工件点,多线程 kNN)
        ↓
Step 5  聚类去游离点        cluster_filter     (欧氏聚类,删除零散小簇)
        ↓
工件点云 workpiece_clean.ply
```

核心思想:`含工件场景点云 - 空环境点云 = 工件点云`。

差分提到统计滤波之前:一方面统计滤波只需作用于差分后的工件点(数量小、速度快),
另一方面统计滤波 + 聚类放在差分之后,正好清理差分残留的背景噪声与游离点。env
点云不做统计滤波,直接用体素结果建 KD-tree 参与差分。完整流程对约 950 万点的输入
约 14 秒完成。

## 目录结构

```
baseline/
├── include/                 # 头文件
│   ├── types.hpp            # 点云类型与 PipelineConfig
│   ├── ply_io.hpp           # PLY 读写(支持 ascii / binary_little_endian)
│   ├── kdtree.hpp           # KD-tree(最近邻 + 有界堆 kNN + 半径搜索)
│   ├── roi_crop.hpp
│   ├── voxel_downsample.hpp
│   ├── diff_segmentation.hpp
│   ├── statistical_filter.hpp
│   ├── cluster_filter.hpp   # 欧氏聚类去游离点
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
clang++ -std=c++17 -O3 -pthread -Iinclude main.cpp src/*.cpp -o baseline
```

或使用 CMake:

```bash
cd baseline && mkdir -p build && cd build
cmake .. && cmake --build .
```

## 使用

支持逐步测试,`--step` 指定运行到哪一步(默认 `all`):

```bash
./baseline --step roi       # 仅 ROI 裁剪
./baseline --step voxel     # 到体素下采样
./baseline --step diff      # 到差分分割
./baseline --step sor       # 到统计滤波
./baseline --step cluster   # 到聚类去游离点(= 完整结果)
./baseline --step all       # 完整流程(默认)
```

各步参数均可命令行传入,调参无需重新编译:

```bash
./baseline --step all --voxel 1.0 --diff-threshold 3 \
    --nb-neighbors 15 --std-ratio 4 \
    --cluster-radius 3 --min-cluster-size 100 \
    --roi -176.4 1164.9 -111.2 960.7 -150 100 \
    --env data/origin-only-environment.ply \
    --scene data/raw_subass_workpiece.ply \
    --output output/baseline
```

中间结果输出到 `output/baseline/`:`env_roi.ply` / `scene_roi.ply`、
`env_voxel.ply` / `scene_voxel.ply`、`workpiece_diff.ply`(差分后)、
`workpiece_sor.ply`(统计滤波后)、`workpiece_clean.ply`(最终结果),
可用 CloudCompare / MeshLab 查看。

## 参数

默认参数在 `baseline/main.cpp` 的 `default_config()` 中,也可用上述命令行选项覆盖(单位:mm):

| 参数 | 命令行 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `roi` | `--roi` | 工作台包围盒 | 基于环境点云工作台层估计 |
| `voxel_size` | `--voxel` | 1.0 | 体素边长 |
| `diff_threshold` | `--diff-threshold` | 3.0 | 差分最近邻距离阈值 |
| `nb_neighbors` | `--nb-neighbors` | 15 | 统计滤波 kNN 邻居数 |
| `std_ratio` | `--std-ratio` | 4.0 | 统计滤波标准差倍数阈值 |
| `cluster_radius` | `--cluster-radius` | 3.0 | 欧氏聚类邻接半径 |
| `min_cluster_size` | `--min-cluster-size` | 100 | 小于该点数的簇视为游离噪声删除 |

## 测试数据

原始工业点云较大,未纳入版本控制。可用脚本生成小规模测试点云:

```bash
python3 scripts/generate_test_data.py
```
