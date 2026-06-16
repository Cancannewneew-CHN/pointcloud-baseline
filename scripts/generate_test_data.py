#!/usr/bin/env python3
"""生成 baseline 测试用 env.ply / scene.ply。"""

from pathlib import Path


def write_ply(path: Path, points):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for x, y, z in points:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")


def grid_points(x_range, y_range, z, step):
    points = []
    x = x_range[0]
    while x <= x_range[1] + 1e-9:
        y = y_range[0]
        while y <= y_range[1] + 1e-9:
            points.append((x, y, z))
            y += step
        x += step
    return points


def block_points(x_range, y_range, z_range, step):
    points = []
    x = x_range[0]
    while x <= x_range[1] + 1e-9:
        y = y_range[0]
        while y <= y_range[1] + 1e-9:
            z = z_range[0]
            while z <= z_range[1] + 1e-9:
                points.append((x, y, z))
                z += step
            y += step
        x += step
    return points


def main():
    root = Path(__file__).resolve().parents[1]
    data_dir = root / "data"

    # 工作台平面 (ROI 内)
    table = grid_points((0.0, 0.5), (0.0, 0.5), 0.0, 0.01)

    # ROI 外背景点
    outside = [
        (0.9, 0.9, 0.0),
        (-0.2, 0.2, 0.0),
        (0.2, 0.9, 0.1),
        (0.5, -0.1, 0.0),
        (1.0, 1.0, 1.0),
    ]

    env_points = table + outside

    # 工件: 小立方体, 仅 scene 有
    workpiece = block_points((0.20, 0.30), (0.20, 0.30), (0.02, 0.08), 0.005)
    scene_points = env_points + workpiece

    write_ply(data_dir / "env.ply", env_points)
    write_ply(data_dir / "scene.ply", scene_points)

    roi_min = (0.10, 0.10, 0.00)
    roi_max = (0.40, 0.40, 0.30)

    def in_roi(p):
        return (
            roi_min[0] <= p[0] <= roi_max[0]
            and roi_min[1] <= p[1] <= roi_max[1]
            and roi_min[2] <= p[2] <= roi_max[2]
        )

    env_roi = sum(1 for p in env_points if in_roi(p))
    scene_roi = sum(1 for p in scene_points if in_roi(p))

    print(f"生成 {data_dir / 'env.ply'}: {len(env_points)} points")
    print(f"生成 {data_dir / 'scene.ply'}: {len(scene_points)} points")
    print(f"ROI 内预期点数: env={env_roi}, scene={scene_roi}")


if __name__ == "__main__":
    main()
