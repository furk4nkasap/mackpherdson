"""Matplotlib gorsellestirmeleri -- Suspension siniflari uzerinden JENERIK
calisir (McPherson/DoubleWishbone/MultiLink icin ayri kod YOK). Her alt
sinif kendi noktalarini/kenarlarini `plot_points()` / `plot_edges()` ile,
roll-center insaasi icin gereken iki hatti `front_view_lines()` ile saglar;
bu modul sadece bunlari cizer.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import kinematics as K


def _set_axes_equal_3d(ax):
    limits = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    centers = limits.mean(axis=1)
    radius = max(0.5 * max(limits[:, 1] - limits[:, 0]), 1.0)
    ax.set_xlim3d([centers[0] - radius, centers[0] + radius])
    ax.set_ylim3d([centers[1] - radius, centers[1] + radius])
    ax.set_zlim3d([centers[2] - radius, centers[2] + radius])


def plot_3d_linkage(susp: K.Suspension):
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection="3d")

    pts = susp.plot_points()
    for name, p in pts.items():
        ax.scatter(*p, s=30)
        ax.text(p[0], p[1], p[2], "  " + name, fontsize=6.5)

    seen_labels = set()
    for p1, p2, color, style, label in susp.plot_edges():
        lbl = label if (label and label not in seen_labels) else None
        if lbl:
            seen_labels.add(lbl)
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]],
                color=color, ls=style, lw=2, label=lbl)

    wc = susp._p("wheel_center")
    r = susp.wheel_radius
    th = np.linspace(0, 2 * np.pi, 60)
    circle = np.stack([wc[0] + r * np.cos(th), np.full_like(th, wc[1]), wc[2] + r * np.sin(th)])
    ax.plot(circle[0], circle[1], circle[2], color="gray", lw=1)

    y_extent = abs(wc[1]) * 1.15 if abs(wc[1]) > 1 else 800
    ys = np.array([0, y_extent]) if wc[1] >= 0 else np.array([y_extent, 0])
    xs = np.array([-400, 400])
    Xg, Yg = np.meshgrid(xs, ys)
    Zg = np.zeros_like(Xg)
    ax.plot_surface(Xg, Yg, Zg, color="lightgray", alpha=0.25)

    ax.set_xlabel("X (ileri) mm")
    ax.set_ylabel("Y (disa) mm")
    ax.set_zlabel("Z (yukari) mm")
    side_tr = "Sag" if str(susp.side).upper().startswith("R") else "Sol"
    axle_tr = "On" if susp.axle == "front" else "Arka"
    ax.set_title(f"{type(susp).__name__} — {axle_tr} {side_tr} — 3D hardpoint'ler")
    ax.legend(loc="upper left", fontsize=7)
    _set_axes_equal_3d(ax)
    fig.tight_layout()
    return fig


def plot_front_view_roll_center(susp: K.Suspension):
    (pA, dA), (pB, dB) = susp.front_view_lines()
    rc = susp.front_view_roll_center()
    ic = rc["instant_center_yz"]
    rc_pt = rc["roll_center_yz"]
    cp = rc["contact_patch_yz"]
    wc_y = susp._p("wheel_center")[1]

    y_lo, y_hi = (min(-0.35 * wc_y, 0), max(1.25 * wc_y, 0)) if wc_y >= 0 else (min(1.25 * wc_y, 0), max(-0.35 * wc_y, 0))
    z_lo, z_hi = -60.0, max(300.0, abs(pA[1]) * 1.4, abs(pB[1]) * 1.4)

    def clip_line(p, d):
        d = np.asarray(d, dtype=float)
        n = np.linalg.norm(d)
        if n < 1e-12:
            return p, p
        d = d / n
        ts = []
        for lo, hi, idx in [(y_lo, y_hi, 0), (z_lo, z_hi, 1)]:
            if abs(d[idx]) > 1e-9:
                ts += [(lo - p[idx]) / d[idx], (hi - p[idx]) / d[idx]]
        if not ts:
            return p, p
        t0, t1 = min(ts), max(ts)
        return p + t0 * d, p + t1 * d

    fig, ax = plt.subplots(figsize=(7, 5))

    p1a, p2a = clip_line(pA, dA)
    ax.plot([p1a[0], p2a[0]], [p1a[1], p2a[1]], "b--", lw=1, label="Line A")
    p1b, p2b = clip_line(pB, dB)
    ax.plot([p1b[0], p2b[0]], [p1b[1], p2b[1]], color="orange", ls="--", lw=1, label="Line B")

    if np.isfinite(ic).all():
        p1c, p2c = clip_line(cp, ic - cp)
        ax.plot([p1c[0], p2c[0]], [p1c[1], p2c[1]], "g--", lw=1, label="Line C (CP → IC)")

    ax.axhline(0, color="k", lw=1)
    ax.axvline(0, color="gray", lw=0.8, ls=":")

    ax.plot(*pA, "o", color="tab:blue", ms=6)
    ax.plot(*pB, "o", color="tab:orange", ms=6)
    ax.plot(*cp, "o", color="black", ms=6)
    ax.annotate("Contact patch", cp, textcoords="offset points", xytext=(5, -12), fontsize=8)

    if np.isfinite(rc_pt).all():
        ax.plot(*rc_pt, "r*", ms=16, label=f"Roll center (h={rc_pt[1]:.1f} mm)")

    ic_txt = f"IC (front view) = ({ic[0]:.0f}, {ic[1]:.0f}) mm" if np.isfinite(ic).all() else "IC tanimsiz (A ∥ B)"
    ax.text(0.02, 0.02, ic_txt, transform=ax.transAxes, fontsize=7.5,
             bbox=dict(boxstyle="round", fc="white", ec="gray", alpha=0.8))

    ax.set_xlabel("Y — disa (mm)")
    ax.set_ylabel("Z — yukari (mm)")
    ax.set_title(f"{type(susp).__name__} — front-view roll-center insaasi")
    ax.legend(loc="upper left", fontsize=7)
    ax.set_xlim(y_lo, y_hi)
    ax.set_ylim(z_lo, z_hi)
    ax.set_aspect("equal", adjustable="box")
    fig.tight_layout()
    return fig


if __name__ == "__main__":
    from kinematics import McPherson, DoubleWishbone, MultiLink
    for name, cls in [("mcpherson", McPherson), ("dw", DoubleWishbone), ("ml", MultiLink)]:
        s = cls(side="R", axle="front")
        plot_3d_linkage(s).savefig(f"/tmp/test_3d_{name}.png", dpi=110)
        plot_front_view_roll_center(s).savefig(f"/tmp/test_rc_{name}.png", dpi=110)
    print("saved test images to /tmp")
