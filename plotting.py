"""Matplotlib visualisations for the McPherson kinematics app.

Kept deliberately dependency-light (matplotlib only, no plotly) so the app
has the smallest possible install footprint.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection

import kinematics as K


def _set_axes_equal_3d(ax):
    """Make 3D axes have equal scale (matplotlib doesn't do this natively)."""
    limits = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    centers = limits.mean(axis=1)
    radius = 0.5 * max(limits[:, 1] - limits[:, 0])
    radius = max(radius, 1.0)
    ax.set_xlim3d([centers[0] - radius, centers[0] + radius])
    ax.set_ylim3d([centers[1] - radius, centers[1] + radius])
    ax.set_zlim3d([centers[2] - radius, centers[2] + radius])


def plot_3d_linkage(hp: K.Hardpoints):
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection="3d")

    pts = {
        "LCA front bushing": hp.lca_front,
        "LCA rear bushing": hp.lca_rear,
        "LCA outer (P1)": hp.lca_outer,
        "Strut top (T)": hp.strut_top,
        "Strut-knuckle ref (S)": hp.strut_knuckle,
        "Tie rod inner (R)": hp.tierod_inner,
        "Tie rod outer (P2)": hp.tierod_outer,
        "Wheel center": hp.wheel_center,
    }
    for name, p in pts.items():
        ax.scatter(*p, s=35)
        ax.text(p[0], p[1], p[2], "  " + name, fontsize=7)

    def line(p1, p2, **kw):
        p1, p2 = np.array(p1), np.array(p2)
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], **kw)

    # LCA (front bushing - outer - rear bushing, forms the arm)
    line(hp.lca_front, hp.lca_outer, color="tab:blue", lw=2, label="LCA")
    line(hp.lca_rear, hp.lca_outer, color="tab:blue", lw=2)
    # Strut tube (visual, top mount to knuckle reference)
    line(hp.strut_top, hp.strut_knuckle, color="tab:orange", lw=2, label="Strut")
    # Kingpin / steering axis, extended to ground
    T, P1, d = K.kingpin_axis(hp)
    ang = K.static_angles(hp)
    gp = ang["kingpin_ground_point"]
    line(T, gp, color="tab:red", lw=1.5, ls="--", label="Kingpin axis (extended)")
    # Tie rod
    line(hp.tierod_inner, hp.tierod_outer, color="tab:green", lw=2, label="Tie rod")
    # Spring/damper (reference)
    line(hp.spring_top, hp.spring_bottom, color="tab:purple", lw=1, ls=":", label="Spring/damper (ref)")

    # Wheel disc (circle in the plane perpendicular to an approximate spin axis = Y)
    wc = np.array(hp.wheel_center)
    r = hp.wheel_radius
    th = np.linspace(0, 2 * np.pi, 60)
    circle = np.stack([wc[0] + r * np.cos(th), np.full_like(th, wc[1]), wc[2] + r * np.sin(th)])
    ax.plot(circle[0], circle[1], circle[2], color="gray", lw=1)

    # ground plane sketch
    ys = np.array([0, wc[1] * 1.15])
    xs = np.array([-400, 400])
    Xg, Yg = np.meshgrid(xs, ys)
    Zg = np.zeros_like(Xg)
    ax.plot_surface(Xg, Yg, Zg, color="lightgray", alpha=0.25)

    ax.set_xlabel("X (fwd) mm")
    ax.set_ylabel("Y (outboard) mm")
    ax.set_zlabel("Z (up) mm")
    ax.set_title("McPherson corner — 3D hardpoints")
    ax.legend(loc="upper left", fontsize=7)
    _set_axes_equal_3d(ax)
    fig.tight_layout()
    return fig


def plot_front_view_roll_center(hp: K.Hardpoints):
    rc = K.front_view_roll_center(hp)
    A = np.array(hp.lca_front)[[1, 2]]
    B = np.array(hp.lca_rear)[[1, 2]]
    lca_pivot = 0.5 * (A + B)
    P1 = np.array(hp.lca_outer)[[1, 2]]
    T = np.array(hp.strut_top)[[1, 2]]
    wc_y = hp.wheel_center[1]
    ic = rc["instant_center_yz"]
    rc_pt = rc["roll_center_yz"]
    cp = rc["contact_patch_yz"]

    # Fixed view window around the points that matter for reading the plot
    # (the IC itself is often far outside this box for near-parallel A/B
    # lines -- that's normal, it's reported in the caption instead of
    # forcing the whole plot to zoom out to it).
    y_lo, y_hi = -0.35 * wc_y, 1.25 * wc_y
    z_lo, z_hi = -60.0, max(300.0, P1[1] * 1.6, T[1] * 0.55)

    def clip_line(p, d):
        """Return two points of the infinite line p+t*d clipped to the view box."""
        d = d / np.linalg.norm(d)
        ts = []
        for lo, hi, idx in [(y_lo, y_hi, 0), (z_lo, z_hi, 1)]:
            if abs(d[idx]) > 1e-9:
                ts += [(lo - p[idx]) / d[idx], (hi - p[idx]) / d[idx]]
        if not ts:
            return p, p
        t0, t1 = min(ts), max(ts)
        return p + t0 * d, p + t1 * d

    fig, ax = plt.subplots(figsize=(7, 5))

    p1a, p2a = clip_line(lca_pivot, P1 - lca_pivot)
    ax.plot([p1a[0], p2a[0]], [p1a[1], p2a[1]], "b--", lw=1, label="Line A (LCA, extended)")

    strut_dir = P1 - T
    perp = np.array([-strut_dir[1], strut_dir[0]])
    p1b, p2b = clip_line(T, perp)
    ax.plot([p1b[0], p2b[0]], [p1b[1], p2b[1]], color="orange", ls="--", lw=1, label="Line B (⊥ strut, through T)")

    if np.isfinite(ic).all():
        p1c, p2c = clip_line(cp, ic - cp)
        ax.plot([p1c[0], p2c[0]], [p1c[1], p2c[1]], "g--", lw=1, label="Line C (CP → IC)")

    ax.axhline(0, color="k", lw=1)
    ax.axvline(0, color="gray", lw=0.8, ls=":")

    ax.plot(*lca_pivot, "bo", ms=6)
    ax.annotate("LCA pivot", lca_pivot, textcoords="offset points", xytext=(5, 5), fontsize=8)
    ax.plot(*P1, "o", color="tab:blue", ms=6)
    ax.annotate("P1 (LCA outer)", P1, textcoords="offset points", xytext=(5, 5), fontsize=8)
    ax.plot(*T, "o", color="tab:orange", ms=6)
    ax.annotate("T (strut top)", T, textcoords="offset points", xytext=(5, 5), fontsize=8)
    ax.plot(*cp, "o", color="black", ms=6)
    ax.annotate("Contact patch", cp, textcoords="offset points", xytext=(5, -12), fontsize=8)

    if np.isfinite(rc_pt).all():
        ax.plot(*rc_pt, "r*", ms=16, label=f"Roll center (h={rc_pt[1]:.1f} mm)")

    ic_txt = f"IC (front view) = ({ic[0]:.0f}, {ic[1]:.0f}) mm" if np.isfinite(ic).all() else "IC undefined (A ∥ B)"
    ax.text(0.02, 0.02, ic_txt, transform=ax.transAxes, fontsize=7.5,
             bbox=dict(boxstyle="round", fc="white", ec="gray", alpha=0.8))

    ax.set_xlabel("Y — outboard (mm)")
    ax.set_ylabel("Z — up (mm)")
    ax.set_title("Front-view roll-center construction")
    ax.legend(loc="upper left", fontsize=7)
    ax.set_xlim(y_lo, y_hi)
    ax.set_ylim(z_lo, z_hi)
    ax.set_aspect("equal", adjustable="box")
    fig.tight_layout()
    return fig


def plot_camber_toe(sweep_result):
    fig, axes = plt.subplots(1, 2, figsize=(9, 4))
    tr = sweep_result["wheel_travel_mm"]
    axes[0].plot(tr, sweep_result["camber_deg"], color="tab:blue")
    axes[0].axvline(0, color="gray", lw=0.8)
    axes[0].axhline(0, color="gray", lw=0.8)
    axes[0].set_xlabel("Wheel travel (mm), + = bump")
    axes[0].set_ylabel("Camber (deg), − = negative camber")
    axes[0].set_title("Camber vs. travel")
    axes[0].grid(alpha=0.3)

    axes[1].plot(tr, sweep_result["toe_deg"], color="tab:green")
    axes[1].axvline(0, color="gray", lw=0.8)
    axes[1].axhline(0, color="gray", lw=0.8)
    axes[1].set_xlabel("Wheel travel (mm), + = bump")
    axes[1].set_ylabel("Toe (deg), + = toe-in")
    axes[1].set_title("Toe vs. travel (bump steer)")
    axes[1].grid(alpha=0.3)

    fig.tight_layout()
    return fig


def plot_roll_center_travel(sweep_result):
    fig, ax = plt.subplots(figsize=(6, 4))
    tr = sweep_result["wheel_travel_mm"]
    ax.plot(tr, sweep_result["roll_center_height_mm"], color="tab:red")
    ax.axvline(0, color="gray", lw=0.8)
    ax.axhline(0, color="gray", lw=0.8)
    ax.set_xlabel("Wheel travel (mm), + = bump")
    ax.set_ylabel("Roll center height (mm)")
    ax.set_title("Roll-center height vs. travel")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    return fig


if __name__ == "__main__":
    hp = K.Hardpoints()
    plot_3d_linkage(hp).savefig("/tmp/test_3d.png", dpi=110)
    plot_front_view_roll_center(hp).savefig("/tmp/test_rc.png", dpi=110)
    res = K.sweep(hp)
    plot_camber_toe(res).savefig("/tmp/test_camber_toe.png", dpi=110)
    plot_roll_center_travel(res).savefig("/tmp/test_rc_travel.png", dpi=110)
    print("saved test images to /tmp")
