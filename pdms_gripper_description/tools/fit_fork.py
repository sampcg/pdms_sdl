"""Solve the fork's link-frame pose on the 320 pro adaptive gripper.

Constraints, all derived rather than guessed:
  * the fork's hinge hole must sit on the joint axis (link origin)
  * the fork's hinge axis (mesh z) must lie along the link's joint axis (link z)
  * the fork's flat plate must face the opposite finger, like the stock blade

That leaves only: which hole, a 180 deg flip, and the spin angle theta about z.
Grid-search those three and score against the stock finger's clamping plane.
"""
import sys, os
import numpy as np

SP = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SP)
import render_urdf as R

SHARE = "/home/samuel/colcon_ws/src/mycobot_ros2/mycobot_description"
R.SHARE = SHARE
STOCK = SHARE + "/urdf/mycobot_320_m5_2022/mycobot_320_m5_2022_adaptive_gripper.urdf"

HOLES = {  # mesh mm, in the fork's own frame
    "left":  {"A": (7.99, 1.28, -0.32), "B": (-14.33, -24.76, -0.32)},
    "right": {"A": (0.00, 0.00, 0.10),  "B": (-22.32, 26.04, 0.10)},
}
FORK = SHARE + "/urdf/pro_adaptive_gripper/gripper_%s1_fork.dae"


def tri_props(V, F):
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    n = np.cross(b - a, c - a)
    area = np.linalg.norm(n, axis=1) / 2.0
    ok = area > 1e-12
    n = n[ok] / (2 * area[ok])[:, None]
    return (a[ok] + b[ok] + c[ok]) / 3.0, n, area[ok]


def dominant_plane(cent, nrm, area, want, tol=0.94):
    """Largest-area planar patch whose outward normal points along `want`."""
    sel = nrm @ want > tol
    if sel.sum() == 0:
        return None
    c, a = cent[sel], area[sel]
    # cluster by offset along `want`
    off = c @ want
    lo, hi = off.min(), off.max()
    bins = np.clip(((off - lo) / max(hi - lo, 1e-9) * 60).astype(int), 0, 59)
    w = np.bincount(bins, weights=a, minlength=60)
    b = int(np.argmax(w))
    m = bins == b
    return (c[m] * a[m, None]).sum(0) / a[m].sum(), w[b]


def rotz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


ROTX180 = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])


def main():
    links, joints = R.load_urdf(STOCK)
    pose = R.fk(links, joints, {"gripper_controller": 0.0})

    # stock fingers in gripper_base frame
    base_inv = np.linalg.inv(pose["gripper_base"])
    stock = {}
    for side in ("left", "right"):
        name = "gripper_%s1" % side
        uri, Tv = links[name]
        V, F = R.load_dae(R.resolve(uri))
        W = base_inv @ pose[name] @ Tv
        P = (W @ np.c_[V, np.ones(len(V))].T).T[:, :3]
        stock[side] = (P, F, base_inv @ pose[name])

    cl = stock["left"][0].mean(0)
    cr = stock["right"][0].mean(0)
    d = cr - cl
    d /= np.linalg.norm(d)
    print("clamping direction (base frame, left->right): ", np.round(d, 4))
    print("stock left  centroid", np.round(cl * 1000, 1), "mm")
    print("stock right centroid", np.round(cr * 1000, 1), "mm")

    target = {}
    for side, want in (("left", d), ("right", -d)):
        P, F, _ = stock[side]
        cent, nrm, area = tri_props(P, F)
        res = dominant_plane(cent, nrm, area, want)
        target[side] = res
        print(f"stock {side:5} clamping-face centroid {np.round(res[0]*1000,1)} mm  area={res[1]*1e6:.0f} mm^2")

    print()
    results = {}
    for side in ("left", "right"):
        want = d if side == "left" else -d
        Vf, Ff = R.load_dae(FORK % side)
        Lpose = stock[side][2]                     # link frame in base frame
        Lrot = Lpose[:3, :3]
        tgt_c, _ = target[side]
        best = None
        for hole in ("A", "B"):
            p = np.array(HOLES[side][hole]) / 1000.0
            for flip in (False, True):
                Rf = ROTX180 if flip else np.eye(3)
                for th in np.arange(0, 2 * np.pi, np.radians(1.0)):
                    Rl = rotz(th) @ Rf
                    t = -Rl @ p
                    Pl = (Rl @ Vf.T).T + t                      # link frame
                    Pb = (Lpose[:3, :3] @ Pl.T).T + Lpose[:3, 3]  # base frame
                    cent, nrm, area = tri_props(Pb, Ff)
                    res = dominant_plane(cent, nrm, area, want, tol=0.90)
                    if res is None:
                        continue
                    c, aw = res
                    ang = 1.0 - float(np.clip((Lrot @ Rl @ np.array([0, 0, 1.0])) @ np.array([0, 0, 1.0]), -1, 1))
                    dist = np.linalg.norm(c - tgt_c)
                    score = dist * 1000 - 0.02 * aw * 1e6       # mm, reward big flat face
                    if best is None or score < best[0]:
                        best = (score, hole, flip, th, dist, aw, c)
        s, hole, flip, th, dist, aw, c = best
        results[side] = (hole, flip, th)
        print(f"{side:5}: hole {hole}  flip={flip}  theta={np.degrees(th):6.1f} deg   "
              f"face-offset={dist*1000:5.1f} mm  face-area={aw*1e6:.0f} mm^2")
        p = np.array(HOLES[side][hole]) / 1000.0
        Rf = ROTX180 if flip else np.eye(3)
        Rl = rotz(th) @ Rf
        t = -Rl @ p
        roll = np.pi if flip else 0.0
        print(f"        -> <origin xyz=\"{t[0]:.5f} {t[1]:.5f} {t[2]:.5f}\" "
              f"rpy=\"{roll:.5f} 0 {th:.5f}\"/>")
    return results


if __name__ == "__main__":
    main()
