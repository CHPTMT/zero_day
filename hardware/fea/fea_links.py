"""Linear static FEA of the two arm links (Group 31 robotic arm).

Each link is meshed with 8-node hexahedral (brick) elements on a regular grid
that follows its real cross-section: a hollow box of 5 mm foam board walls,
with the three side windows cut in the second arm. Each link is clamped where
the servo horn grips it and loaded with its own weight, everything it carries
and the button-press reaction, all with a dynamic factor.

The solver is checked against Euler-Bernoulli beam theory before use.

Material (assumed, the arm is white 5 mm foam board; change below if known):
expanded PVC foam board, E = 800 MPa, nu = 0.35, density 500 kg/m3,
tensile strength 15 MPa.
"""
import json, math, os, sys, time
import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

E, NU, RHO, STRENGTH = 800.0, 0.35, 500e-9, 15.0   # MPa, -, kg/mm3 (500 kg/m3), MPa
G = 9.81e-3            # N per gram  (1 g -> 0.00981 N)
DYN = 2.0              # dynamic factor on all weights (accelerations, jerks)
OUT = os.path.dirname(os.path.abspath(__file__))


def D_matrix(E, nu):
    l = E * nu / ((1 + nu) * (1 - 2 * nu)); m = E / (2 * (1 + nu))
    D = np.zeros((6, 6))
    D[:3, :3] = l; D[np.arange(3), np.arange(3)] += 2 * m
    D[3:, 3:] = np.eye(3) * m
    return D


def hex_B(xi, eta, zeta, h):
    s = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
                  [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]], float)
    dN = np.zeros((8, 3))
    for a in range(8):
        sx, sy, sz = s[a]
        dN[a] = [sx * (1 + sy * eta) * (1 + sz * zeta), sy * (1 + sx * xi) * (1 + sz * zeta),
                 sz * (1 + sx * xi) * (1 + sy * eta)]
    dN *= 1 / 8 * 2 / h               # d/dx = d/dxi * 2/h
    B = np.zeros((6, 24))
    for a in range(8):
        dx, dy, dz = dN[a]
        B[0, 3 * a] = dx; B[1, 3 * a + 1] = dy; B[2, 3 * a + 2] = dz
        B[3, 3 * a] = dy; B[3, 3 * a + 1] = dx
        B[4, 3 * a + 1] = dz; B[4, 3 * a + 2] = dy
        B[5, 3 * a] = dz; B[5, 3 * a + 2] = dx
    return B


def hex_K(h, D):
    g = 1 / math.sqrt(3); K = np.zeros((24, 24))
    for xi in (-g, g):
        for eta in (-g, g):
            for zeta in (-g, g):
                B = hex_B(xi, eta, zeta, h)
                K += B.T @ D @ B * (h / 2) ** 3
    return K


def solve(occ, h, fixed_x, loads, body_factor, label, couple=None):
    """occ[i,j,k] voxel grid (x along the link). fixed_x: clamp nodes with x <= fixed_x.
    loads: list of (selector(x,y,z)->bool over nodes, force vector N total)."""
    t0 = time.time()
    nx, ny, nz = occ.shape
    D = D_matrix(E, NU); Ke = hex_K(h, D)
    el = np.argwhere(occ)
    corner = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]])
    gid = lambda ijk: (ijk[..., 0] * (ny + 1) + ijk[..., 1]) * (nz + 1) + ijk[..., 2]
    conn_full = gid(el[:, None, :] + corner[None, :, :])
    used, conn = np.unique(conn_full, return_inverse=True)
    conn = conn.reshape(-1, 8)
    nn = len(used)
    ii = used // ((ny + 1) * (nz + 1)); jj = (used // (nz + 1)) % (ny + 1); kk = used % (nz + 1)
    X = np.stack([ii * h, jj * h, kk * h], 1)
    dofs = (conn[:, :, None] * 3 + np.arange(3)[None, None, :]).reshape(-1, 24)
    rows = np.repeat(dofs, 24, axis=1).ravel(); cols = np.tile(dofs, (1, 24)).ravel()
    K = sp.coo_matrix((np.tile(Ke.ravel(), len(el)), (rows, cols)), shape=(3 * nn, 3 * nn)).tocsr()
    F = np.zeros(3 * nn)
    # body load: own weight, -z, with the dynamic factor, split over the 8 nodes
    w_el = RHO * h ** 3 * 9.81 * body_factor        # N per element
    np.add.at(F, conn.ravel() * 3 + 2, -w_el / 8)
    for sel, f in loads:
        idx = np.where(sel(X[:, 0], X[:, 1], X[:, 2]))[0]
        for c in range(3):
            np.add.at(F, idx * 3 + c, f[c] / len(idx))
    if couple is not None:   # bending moment M about y, as a linear axial traction over the selected face
        sel, M, zc = couple
        idx = np.where(sel(X[:, 0], X[:, 1], X[:, 2]))[0]
        zz = X[idx, 2] - zc
        np.add.at(F, idx * 3, M * zz / np.sum(zz ** 2))
    fixed = np.where(X[:, 0] <= fixed_x + 1e-9)[0]
    fdofs = np.concatenate([fixed * 3, fixed * 3 + 1, fixed * 3 + 2])
    free = np.setdiff1d(np.arange(3 * nn), fdofs)
    u = np.zeros(3 * nn)
    u[free] = spla.spsolve(K[free][:, free].tocsc(), F[free])
    Bc = hex_B(0, 0, 0, h)
    strain = (Bc @ u[dofs].T).T
    sig = strain @ D.T
    sx, sy, sz, txy, tyz, tzx = sig.T
    vm = np.sqrt(0.5 * ((sx - sy) ** 2 + (sy - sz) ** 2 + (sz - sx) ** 2) + 3 * (txy ** 2 + tyz ** 2 + tzx ** 2))
    cent = (el + 0.5) * h
    U = u.reshape(-1, 3)
    mass_g = len(el) * h ** 3 * RHO * 1e3
    print(f"{label}: {len(el)} elements, {nn} nodes, {3*nn} dof, solved in {time.time()-t0:.1f} s, link mass {mass_g:.0f} g")
    return dict(X=X, U=U, cent=cent, vm=vm, sx=sx, el=el, mass_g=mass_g, nel=len(el), ndof=3 * nn)


def vm_from_U(r, h):
    """von Mises at element centres from a (combined) displacement field."""
    nx, ny, nz = r["el"].max(axis=0) + 2
    D = D_matrix(E, NU); Bc = hex_B(0, 0, 0, h)
    corner = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]])
    key = {tuple(np.round(x / h).astype(int)): n for n, x in enumerate(r["X"])}
    conn = np.array([[key[tuple(e + c)] for c in corner] for e in r["el"]])
    ue = r["U"][conn].reshape(len(conn), 24)
    sig = (Bc @ ue.T).T @ D.T
    sx, sy, sz, txy, tyz, tzx = sig.T
    return np.sqrt(0.5 * ((sx - sy) ** 2 + (sy - sz) ** 2 + (sz - sx) ** 2) + 3 * (txy ** 2 + tyz ** 2 + tzx ** 2))


def box_occ(length_total, w, t, wall, h, windows=(), L_pivot=None, end=22):
    """Voxels of a hollow box: x along the link (length_total), y thickness t, z side-face width w."""
    nx, ny, nz = int(round(length_total / h)), int(round(t / h)), int(round(w / h))
    x = (np.arange(nx) + 0.5) * h; y = (np.arange(ny) + 0.5) * h; z = (np.arange(nz) + 0.5) * h
    Xg, Yg, Zg = np.meshgrid(x, y, z, indexing="ij")
    shell = (Yg < wall) | (Yg > t - wall) | (Zg < wall) | (Zg > w - wall)
    occ = shell.copy()
    for f in windows:   # windows through the two side faces (the y walls)
        xc = end + f * L_pivot
        win = (np.abs(Xg - xc) < 11) & (np.abs(Zg - w / 2) < 12) & ((Yg < wall) | (Yg > t - wall))
        occ &= ~win
    return occ


def verify():
    """Solid 20 x 20 x 200 mm cantilever, 1 N tip load: compare with beam theory."""
    h = 2.5
    occ = np.ones((80, 8, 8), bool)
    L = 200.0; P = 1.0; I = 20 * 20 ** 3 / 12
    r = solve(occ, h, 0.0, [(lambda x, y, z: x >= L - 1e-9, (0, 0, -P))], 0.0, "check: solid cantilever")
    tip = r["U"][r["X"][:, 0] >= L - 1e-9, 2].mean()
    Gs = E / (2 * (1 + NU)); A = 400
    beam = -(P * L ** 3 / (3 * E * I) + P * L / (5 / 6 * Gs * A))
    s_beam = P * L * 10 / I
    near = (r["cent"][:, 0] > 60) & (r["cent"][:, 0] < 70)
    s_fe = np.abs(r["sx"][near]).max()
    print(f"  tip deflection FE {tip:.4f} mm, beam theory {beam:.4f} mm ({100*(tip/beam-1):+.1f}%)")
    return dict(tip_fe=float(tip), tip_beam=float(beam), err_pct=float(100 * (tip / beam - 1)),
                stress_fe_at_65mm=float(s_fe), stress_beam_at_65mm=float(P * (L - 65) * 10 / I))


def plot(r, fname, title, scale):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 3.2), dpi=150)
    # side view: max von Mises through the thickness at each (x, z) cell
    el, vm = r["el"], r["vm"]
    nxm, nzm = el[:, 0].max() + 1, el[:, 2].max() + 1
    img = np.full((nzm, nxm), np.nan)
    for (i, j, k), v in zip(el, vm):
        if np.isnan(img[k, i]) or v > img[k, i]:
            img[k, i] = v
    h = r["h"]
    im = ax.imshow(img, origin="lower", extent=[0, nxm * h, 0, nzm * h], cmap="viridis", aspect="equal")
    cb = fig.colorbar(im, ax=ax, fraction=0.025, pad=0.02); cb.set_label("von Mises stress (MPa)")
    ax.set_xlabel("distance along the link (mm)"); ax.set_ylabel("mm")
    ax.set_title(title, fontname="DejaVu Serif", fontsize=10)
    fig.tight_layout(); fig.savefig(fname); plt.close(fig)


def main():
    res = {"material": dict(E_MPa=E, nu=NU, density_kg_m3=round(RHO * 1e9), strength_MPa=STRENGTH), "dynamic_factor": DYN}
    res["verification"] = verify()
    h = 2.0
    # ---- second arm: cantilever from the elbow horn, horizontal (worst for bending)
    END = 22; L2 = 160
    occ2 = box_occ(L2 + 2 * END, 40, 20, 4, h, windows=(0.30, 0.52, 0.74), L_pivot=L2, end=END)
    head_g = 46.0                     # sensor head with sensors, pusher, wires (estimated)
    press_N = 0.177 / 0.184           # SG90 stall torque 0.177 N m at the 184 mm pusher lever
    tip = lambda x, y, z: x >= END + L2 - 4
    case_a = solve(occ2, h, END + 6, [(tip, (0, 0, -DYN * head_g * G))], DYN, "second arm, case A (weight x2)")
    # case B: pressing. The button pushes back up on the pusher tip with up to the
    # elbow servo's stall force while the arm's own weight acts at x1.
    case_b = solve(occ2, h, END + 6, [(tip, (0, 0, -head_g * G + press_N))], 1.0, "second arm, case B (press)")
    case_a["h"] = case_b["h"] = h
    # ---- main arm: clamped at the shoulder horns, leaning 50 deg forward (its limit),
    #      second arm level and fully out, so the elbow carries a force and a moment
    L1 = 225; w2_g = case_a["mass_g"]; servo_g = 9.0
    W = DYN * (w2_g + head_g + servo_g) * G                 # N at the elbow
    M = DYN * (w2_g * G * (L2 / 2) + head_g * G * L2)        # N mm, second arm level
    phi = math.radians(50)
    h1 = 2.5
    occ1 = box_occ(L1 + 2 * END, 50, 30, 5, h1)
    end1 = lambda x, y, z: x >= END + L1 - 5
    face = lambda x, y, z: x >= x.max() - 1e-6        # the elbow end face
    # In the link frame x runs along the arm; gravity has an axial part (cos phi)
    # and a bending part (sin phi). Body weight is applied the same way below.
    r_comb = solve(occ1, h1, END + 8, [(end1, (-W * math.cos(phi), 0, -W * math.sin(phi)))], 0.0,
                   "main arm, elbow loads", couple=(face, M, 25.0))   # + = top fibre in tension, same sense as the elbow force
    # own weight of the main arm, tilted: equivalent nodal loads by a second solve
    mass1 = r_comb["mass_g"]
    Wself = DYN * mass1 * G
    r_self = solve(occ1, h1, END + 8, [(lambda x, y, z: x >= 0, (-Wself * math.cos(phi), 0, -Wself * math.sin(phi)))], 0.0, "main arm, own weight")
    # combine (linear): displacements add; von Mises is recomputed from the summed field
    r_comb["U"] = r_comb["U"] + r_self["U"]
    r_comb["h"] = h1
    r_comb["vm"] = vm_from_U(r_comb, h1)
    r_comb["mass_g"] = mass1

    def summary(r, L_end, label):
        away = (r["cent"][:, 0] > END + 15) & (r["cent"][:, 0] < L_end - 10)   # skip clamp and load patches
        tipd = np.abs(r["U"][r["X"][:, 0] >= L_end - 4 - 1e-9]).max(axis=0)
        s_max = float(r["vm"][away].max()); s_peak = float(r["vm"].max())
        return dict(label=label, elements=int(r["nel"]), dof=int(r["ndof"]), link_mass_g=round(r["mass_g"], 1),
                    max_von_mises_MPa=round(s_max, 3), peak_at_clamp_MPa=round(s_peak, 3),
                    safety_factor=round(STRENGTH / s_max, 1), tip_deflection_mm=round(float(np.linalg.norm(tipd)), 3))
    res["second_arm_case_A"] = summary(case_a, END + L2, "Second arm, own weight and head x2")
    res["second_arm_case_B"] = summary(case_b, END + L2, "Second arm pressing: weight x1 with the full button-press reaction")
    res["main_arm"] = summary(r_comb, END + L1, "Main arm at 50 deg, second arm level, all weights x2")
    res["loads"] = dict(head_g=head_g, press_reaction_N=round(press_N, 3), elbow_force_N=round(W, 3), elbow_moment_Nmm=round(M, 1))
    plot(case_a, os.path.join(OUT, "fea_second_arm.png"), "Second arm, von Mises stress (MPa), weights x2, clamped at the elbow horn (left)", 1)
    plot(r_comb, os.path.join(OUT, "fea_main_arm.png"), "Main arm at 50 deg forward, von Mises stress (MPa), clamped at the shoulder horns (left)", 1)
    with open(os.path.join(OUT, "fea_results.json"), "w") as f:
        json.dump(res, f, indent=2)
    print(json.dumps(res, indent=2))


if __name__ == "__main__":
    main()
