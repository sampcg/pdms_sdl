"""Offscreen render of the mycobot adaptive-gripper subtree straight from a URDF.

Loads COLLADA meshes (honouring <unit> scale and visual-scene node transforms),
runs FK over the gripper links, and paints the triangles with a z-sorted
painter's algorithm. No trimesh/pycollada needed.
"""
import sys, os, re
import numpy as np
import xml.etree.ElementTree as ET
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SHARE = None  # filled in from argv


# ---------------------------------------------------------------- COLLADA ---
def _floats(t):
    return np.array([float(x) for x in t.split()])


def load_dae(path):
    """Return (V, F) in metres, with node transforms and unit scale applied."""
    root = ET.parse(path).getroot()
    NS = root.tag.split('}')[0].strip('{')
    ns = {'c': NS}
    q = lambda tag: '{%s}%s' % (NS, tag)

    unit = root.find('.//c:unit', ns)
    scale = float(unit.get('meter')) if unit is not None else 1.0

    srcs = {s.get('id'): s for s in root.iter(q('source'))}

    # geometry id -> (verts, faces)
    geoms = {}
    for g in root.iter(q('geometry')):
        m = g.find('c:mesh', ns)
        if m is None:
            continue
        vt = m.find('c:vertices', ns)
        if vt is None:
            continue
        vid = vt.get('id')
        pid = [i.get('source')[1:] for i in vt.findall('c:input', ns)
               if i.get('semantic') == 'POSITION'][0]
        V = _floats(srcs[pid].find('c:float_array', ns).text).reshape(-1, 3)

        faces = []
        for prim in list(m.findall('c:triangles', ns)) + list(m.findall('c:polylist', ns)):
            ins = prim.findall('c:input', ns)
            stride = max(int(i.get('offset', 0)) for i in ins) + 1
            off = [int(i.get('offset', 0)) for i in ins if i.get('semantic') == 'VERTEX']
            if not off:
                continue
            p = prim.find('c:p', ns)
            if p is None:
                continue
            idx = np.array([int(x) for x in p.text.split()]).reshape(-1, stride)[:, off[0]]
            vc = prim.find('c:vcount', ns)
            if vc is not None:                       # polylist: fan-triangulate
                counts = [int(x) for x in vc.text.split()]
                pos = 0
                for c in counts:
                    poly = idx[pos:pos + c]; pos += c
                    for k in range(1, c - 1):
                        faces.append([poly[0], poly[k], poly[k + 1]])
            else:
                faces.append(idx.reshape(-1, 3))
        if not len(faces):
            continue
        F = np.vstack([f if np.ndim(f) == 2 else np.array([f]) for f in faces])
        geoms[g.get('id')] = (V, F)

    # walk the visual scene so node transforms are applied
    out_v, out_f = [], []

    def walk(node, M):
        T = M.copy()
        for ch in node:
            tag = ch.tag.split('}')[-1]
            if tag == 'matrix':
                T = T @ _floats(ch.text).reshape(4, 4)
            elif tag == 'translate':
                t = np.eye(4); t[:3, 3] = _floats(ch.text)[:3]; T = T @ t
            elif tag == 'rotate':
                v = _floats(ch.text); ax, ang = v[:3], np.radians(v[3])
                n = np.linalg.norm(ax)
                if n > 1e-12:
                    ax = ax / n
                    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
                    R = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)
                    r = np.eye(4); r[:3, :3] = R; T = T @ r
            elif tag == 'scale':
                s = np.eye(4); s[[0, 1, 2], [0, 1, 2]] = _floats(ch.text)[:3]; T = T @ s
        for ch in node:
            tag = ch.tag.split('}')[-1]
            if tag == 'instance_geometry':
                gid = ch.get('url')[1:]
                if gid in geoms:
                    V, F = geoms[gid]
                    Vh = (T @ np.c_[V, np.ones(len(V))].T).T[:, :3]
                    out_f.append(F + sum(len(x) for x in out_v))
                    out_v.append(Vh)
            elif tag == 'node':
                walk(ch, T)

    scenes = list(root.iter(q('visual_scene')))
    if scenes:
        for scene in scenes:
            for n in scene:
                if n.tag.split('}')[-1] == 'node':
                    walk(n, np.eye(4))
    if not out_v:                                     # no visual scene: raw geometry
        for V, F in geoms.values():
            out_f.append(F + sum(len(x) for x in out_v))
            out_v.append(V)
    if not out_v:
        return np.zeros((0, 3)), np.zeros((0, 3), int)
    return np.vstack(out_v) * scale, np.vstack(out_f)


# ------------------------------------------------------------------- URDF ---
def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr]])


def xform(el):
    T = np.eye(4)
    if el is None:
        return T
    o = el.find('origin')
    if o is None:
        return T
    xyz = _floats(o.get('xyz', '0 0 0'))
    rpy = _floats(o.get('rpy', '0 0 0'))
    T[:3, :3] = rpy_mat(*rpy)
    T[:3, 3] = xyz
    return T


def load_urdf(path):
    r = ET.parse(path).getroot()
    links, joints = {}, []
    for l in r.findall('link'):
        vis = l.find('visual')
        mesh = None
        if vis is not None:
            g = vis.find('geometry/mesh')
            if g is not None:
                mesh = g.get('filename')
        links[l.get('name')] = (mesh, xform(vis))
    for j in r.findall('joint'):
        ax = j.find('axis')
        joints.append(dict(
            name=j.get('name'), type=j.get('type'),
            parent=j.find('parent').get('link'), child=j.find('child').get('link'),
            T=xform(j), axis=_floats(ax.get('xyz')) if ax is not None else np.array([0, 0, 1.]),
            mimic=j.find('mimic')))
    return links, joints


def fk(links, joints, q):
    """q: dict joint-name -> angle. Returns link-name -> 4x4 world transform."""
    pose = {}
    roots = {j['child'] for j in joints}
    for n in links:
        if n not in roots:
            pose[n] = np.eye(4)
    changed = True
    while changed:
        changed = False
        for j in joints:
            if j['child'] in pose or j['parent'] not in pose:
                continue
            a = 0.0
            if j['type'] in ('revolute', 'continuous'):
                if j['mimic'] is not None:
                    m = j['mimic']
                    a = q.get(m.get('joint'), 0.0) * float(m.get('multiplier', 1)) \
                        + float(m.get('offset', 0))
                else:
                    a = q.get(j['name'], 0.0)
            ax = j['axis'] / (np.linalg.norm(j['axis']) or 1)
            K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
            R = np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * (K @ K)
            Rj = np.eye(4); Rj[:3, :3] = R
            pose[j['child']] = pose[j['parent']] @ j['T'] @ Rj
            changed = True
    return pose


def resolve(uri):
    return os.path.join(SHARE, uri.split('mycobot_description/')[-1])


# ----------------------------------------------------------------- render ---
GRIP = ['gripper_base', 'gripper_left1', 'gripper_left2', 'gripper_left3',
        'gripper_right1', 'gripper_right2', 'gripper_right3']
HILITE = {'gripper_left1': '#e8564a', 'gripper_right1': '#f0a030'}


def render(ax, urdf_path, q, view, title):
    links, joints = load_urdf(urdf_path)
    pose = fk(links, joints, q)
    cache = {}
    tris, cols = [], []
    for name in GRIP:
        if name not in links or name not in pose:
            continue
        uri, Tv = links[name]
        if not uri:
            continue
        f = resolve(uri)
        if f not in cache:
            cache[f] = load_dae(f)
        V, F = cache[f]
        if not len(V):
            continue
        T = pose[name] @ Tv
        W = (T @ np.c_[V, np.ones(len(V))].T).T[:, :3]
        tris.append(W[F])
        cols += [HILITE.get(name, '#c9ccd1')] * len(F)
    if not tris:
        ax.set_title(title + '  [EMPTY]'); return
    P = np.vstack(tris); cols = np.array(cols)

    # camera
    el, az = np.radians(view[0]), np.radians(view[1])
    fwd = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    up = np.array([0, 0, 1.0])
    right = np.cross(fwd, up); right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    B = np.stack([right, up, fwd])
    C = P @ B.T                                        # (n,3,3) camera coords

    depth = C[:, :, 2].mean(1)
    order = np.argsort(-depth)
    n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
    ln = np.linalg.norm(n, axis=1); ln[ln == 0] = 1
    shade = np.clip(np.abs(n @ np.array([0.4, 0.5, 0.75])) / ln, 0.25, 1.0)

    from matplotlib.collections import PolyCollection
    import matplotlib.colors as mc
    base = np.array([mc.to_rgb(c) for c in cols])
    rgb = np.clip(base * (0.45 + 0.55 * shade[:, None]), 0, 1)
    pc = PolyCollection(C[order][:, :, :2], facecolors=rgb[order],
                        edgecolors='none', antialiaseds=False)
    ax.add_collection(pc)
    ax.autoscale_view()
    ax.set_xlim(C[:, :, 0].min(), C[:, :, 0].max())
    ax.set_ylim(C[:, :, 1].min(), C[:, :, 1].max())
    ax.set_aspect('equal'); ax.axis('off')
    ax.set_title(title, fontsize=10)


if __name__ == '__main__':
    SHARE = sys.argv[1]
    outfile = sys.argv[2]
    variants = sys.argv[3:]
    views = [(12, -90, 'front'), (12, 0, 'side')]
    fig, axes = plt.subplots(len(views), len(variants),
                             figsize=(4.0 * len(variants), 4.2 * len(views)),
                             squeeze=False)
    for c, v in enumerate(variants):
        name = os.path.basename(v).replace('mycobot_320_pi_2022_adaptive_gripper', '').replace('.urdf', '') or 'stock'
        for r, (el, az, vn) in enumerate(views):
            render(axes[r][c], v, {'gripper_controller': 0.0}, (el, az),
                   f'{name}  [{vn}]')
    fig.patch.set_facecolor('white')
    plt.tight_layout()
    plt.savefig(outfile, dpi=115)
    print(outfile)
