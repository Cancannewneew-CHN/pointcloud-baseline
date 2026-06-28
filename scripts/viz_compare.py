import os
os.environ.setdefault("MPLCONFIGDIR", "/tmp/mplcfg")
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_ply(path):
    with open(path, "rb") as f:
        assert f.readline().strip() == b"ply"
        fmt = f.readline().split()[1]
        binary = fmt.startswith(b"binary")
        n = 0
        hdr = 2
        while True:
            l = f.readline()
            hdr += 1
            t = l.split()
            if t and t[0] == b"element" and t[1] == b"vertex":
                n = int(t[2])
            elif t and t[0] == b"end_header":
                break
        if n == 0:
            return np.zeros((0, 3))
        if binary:
            return np.frombuffer(f.read(n * 12), dtype="<f4").reshape(n, 3)
        return np.loadtxt(path, skiprows=hdr)[:, :3]


def subsample(xyz, n=40000):
    if len(xyz) <= n:
        return xyz
    idx = np.random.default_rng(0).choice(len(xyz), n, replace=False)
    return xyz[idx]


def main():
    names = ["wkpc1", "wkpc2", "wkpc3"]
    methods = ["ransac", "zcut"]

    fig = plt.figure(figsize=(16, 9))
    for col, name in enumerate(names):
        for row, method in enumerate(methods):
            path = f"output/{name}_{method}/05_workpiece_clean.ply"
            xyz = load_ply(path)
            full = len(xyz)
            xyz = subsample(xyz)
            ax = fig.add_subplot(2, 3, row * 3 + col + 1, projection="3d")
            ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c=xyz[:, 2],
                       cmap="viridis", s=1.2, linewidths=0)
            ax.set_title(f"{name} / {method}  ({full} pts)", fontsize=10)
            ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
            mx = xyz.max(0); mn = xyz.min(0); ctr = (mx + mn) / 2
            r = (mx - mn).max() / 2
            ax.set_xlim(ctr[0]-r, ctr[0]+r)
            ax.set_ylim(ctr[1]-r, ctr[1]+r)
            ax.set_zlim(ctr[2]-r, ctr[2]+r)
            ax.view_init(elev=25, azim=-60)
    fig.suptitle("RANSAC (top) vs zcut (bottom) - workpiece_clean, colored by z", fontsize=13)
    fig.tight_layout()
    out = "output/viz_compare.png"
    fig.savefig(out, dpi=130)
    print("saved", out)


if __name__ == "__main__":
    main()
