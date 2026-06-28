# 点云预处理 Baseline

基于传统几何方法的工业点云预处理与工件提取 pipeline(参考东北大学《基于 3D 点云的平面角接焊缝特征提取与运动跟踪》)。

从工业 3D 相机采集的原始点云中去除背景与噪声,提取出工件点云,供后续焊缝特征提取使用。

## 两条处理分支

根据是否有空环境点云,提供两个可执行程序,后半段流程(体素 → 统计滤波 → 聚类)完全相同,仅背景剔除方式不同:

| 分支 | 程序 | 适用场景 | 背景剔除 | 输入格式 |
| --- | --- | --- | --- | --- |
| **A 差分** | `baseline` | 有空环境点云 | 场景点云 − 空环境点云 | PLY |
| **B 去台面** | `tabletop` | 无环境点云,工件在工作台上 | 去除工作台平面 | PCD / PLY |

> GUI 集成方式见 [`docs/GUI集成接口.md`](docs/GUI集成接口.md),参数详解见 [`docs/参数配置说明.md`](docs/参数配置说明.md)。

## 分支 A:差分方案(`baseline`)

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
│   ├── pcd_io.hpp           # PCD 读取(binary / ascii)
│   ├── kdtree.hpp           # KD-tree(最近邻 + 有界堆 kNN + 半径搜索)
│   ├── roi_crop.hpp
│   ├── voxel_downsample.hpp
│   ├── diff_segmentation.hpp # 分支 A:差分背景剔除
│   ├── plane_filter.hpp      # 分支 B:去工作台平面(RANSAC / zcut)
│   ├── statistical_filter.hpp
│   ├── cluster_filter.hpp   # 欧氏聚类去游离点
│   └── pipeline.hpp         # 分支 A 流程编排
├── src/                     # 各模块实现(一模块一文件)
├── main.cpp                 # 分支 A 入口(差分),支持分步测试
├── tabletop_main.cpp        # 分支 B 入口(去台面)
└── CMakeLists.txt
docs/                        # 设计/参数/GUI 集成文档
scripts/
├── generate_test_data.py    # 生成测试用小点云
└── viz_result.py            # 结果点云可视化
```

## 构建

需要 C++17 编译器。手动编译两个程序:

```bash
cd baseline
clang++ -std=c++17 -O3 -pthread -Iinclude main.cpp src/*.cpp -o baseline           # 分支 A
clang++ -std=c++17 -O3 -pthread -Iinclude tabletop_main.cpp src/*.cpp -o tabletop   # 分支 B
```

或使用 CMake(同时构建 `baseline` 与 `tabletop`):

```bash
cd baseline && mkdir -p build && cd build
cmake .. && cmake --build .
```

## 分支 A 使用(差分)

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

## 分支 A 参数

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

## 分支 B:去台面方案(`tabletop`)

无空环境点云时使用,以"去除工作台平面"代替差分。流程:
`读取 → [可选 ROI] → 去台面 → 体素下采样 → 统计滤波 → 聚类`。

去台面方法**根据是否提供 z 值自动选择**:

- 给了 `--z-cut <mm>` → **zcut**:直接保留 `z > 该值`(简单、快)。
- 没给 → **ransac**:自动拟合最大近水平平面并去除,**无需任何高度信息**;可选 `--table-z` 作为提示加速。

```bash
# 不填 z → RANSAC 自动去台面
./tabletop --input data/.../wkpc1/merged_pointcloud.pcd --output output/wkpc1_ransac

# 填 z=-42 → zcut
./tabletop --input data/.../wkpc1/merged_pointcloud.pcd --z-cut -42 --output output/wkpc1_zcut
```

| 参数 | 命令行 | 默认值 | 说明 |
| --- | --- | --- | --- |
| 输入点云 | `--input` | 必填 | .pcd / .ply |
| z 切割高度 | `--z-cut` | 不填 | 填了→zcut;不填→ransac |
| 台面高度提示 | `--table-z` | 不填 | 仅 ransac,可选 |
| 平面厚度 | `--plane-thickness` | 5.0 | 仅 ransac |
| 体素 / 统计滤波 / 聚类 | 同分支 A | — | `--voxel` `--std-ratio` `--keep-largest` 等 |

中间结果输出:`02_no_table.ply` / `03_voxel.ply` / `04_sor.ply` / `05_workpiece_clean.ply`。

## 测试数据

原始工业点云较大,未纳入版本控制。可用脚本生成小规模测试点云:

```bash
python3 scripts/generate_test_data.py
```
