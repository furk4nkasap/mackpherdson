"""
McPherson strut suspension kinematics.

Coordinate system (SAE-ish, single corner, e.g. front-right):
    X : forward   (+)
    Y : outboard  (+)   (toward the wheel, away from vehicle centerline)
    Z : up        (+)
Units: millimetres, degrees for angles unless noted.

RIGID-BODY MODEL
-----------------
The knuckle (upright) is treated as one rigid body carrying three points that
are fixed relative to it:

    P1 = LCA outer ball joint
    P2 = tie-rod outer ball joint
    S  = strut-to-knuckle clamp reference point

Three linkages constrain the knuckle:

1. Lower control arm (LCA): a rigid link with two chassis-side revolute
   bushings (front A, rear B) and a ball joint at P1. This constrains P1 to
   move on a circle around the A-B axis. The angle theta on that circle is
   the sweep's DRIVING variable (this is what "wheel travel" is derived
   from).

2. Strut: a spherical joint at the fixed chassis point T (top mount),
   telescoping down to a rigid clamp on the knuckle at S. Telescoping only
   allows sliding along the strut's own axis, so the knuckle-fixed line
   through S (with a direction that is fixed *in the knuckle's own frame*)
   must always pass through T. That is 2 independent scalar constraints
   (a line through a fixed point).

3. Tie rod: a rigid link of fixed length between the fixed rack point R and
   P2. Steering is held fixed (R does not move) for a pure bump/rebound
   sweep -> 1 scalar constraint.

After placing P1 from the LCA circle, the knuckle still has 3 rotational
DOF about P1. The strut (2 eqns) + tie rod (1 eqn) constraints exactly
determine that rotation, and it is solved numerically (least squares,
warm-started from the previous step) for every theta in the sweep.

It can be shown (see README) that with P1 and T both fixed, the family of
knuckle orientations satisfying constraint (2) is exactly rotation about the
T-P1 line -- i.e. the classic kingpin/steering axis -- and the tie rod picks
the one member of that family that also matches its own fixed length. This
is why KPI/caster/scrub/trail below are computed directly from T and P1,
while the strut's own reference point S only matters to keep the numerical
solver constrained during a bump/rebound sweep with steering locked.

ASSUMPTIONS (also read the README):
- All links treated as rigid (no bushing compliance).
- Steering held fixed for the bump/rebound sweep.
- LCA idealised as a single rigid arm rotating about the line through its
  two inboard bushings.
- Camber/toe are reported as the knuckle's rotation about the vehicle-fixed
  X / Z axes (extrinsic XYZ Euler decomposition of the rotation relative to
  the static position). This is the standard practical simplification used
  by hand/quick kinematic tools; it is essentially exact for small angles
  and a good approximation over typical +/-80-100 mm bump/rebound sweeps.
- Anti-dive/anti-squat uses a simplified side-view swing-arm angle (LCA
  virtual pivot -> P1, side view) rather than a full side-view instant-
  center linkage solve. Treat it as indicative, not exact.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields
from typing import List, Tuple

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

Vec3 = Tuple[float, float, float]


@dataclass
class Hardpoints:
    """Illustrative default geometry for a generic C-segment, front-right
    McPherson corner (mm). NOT taken from any real vehicle -- replace with
    your own CAD/Adams hardpoints. Chosen to give plausible target-range
    output (KPI ~11 deg, caster ~5 deg, small scrub radius, front-view RC
    a few tens of mm, anti-dive ~20%) so the app looks sane out of the box.
    """
    # --- LCA (lower control arm) ---
    lca_front: Vec3 = (-60.0, 225.0, 115.0)   # inboard front bushing (chassis)
    lca_rear: Vec3 = (-300.0, 245.0, 109.0)   # inboard rear bushing (chassis)
    lca_outer: Vec3 = (20.0, 725.0, 120.0)    # outer ball joint (knuckle) = P1

    # --- Strut ---
    strut_top: Vec3 = (-31.0, 612.0, 700.0)     # top mount (chassis) = T
    strut_knuckle: Vec3 = (10.0, 715.0, 330.0)  # clamp reference point (knuckle) = S

    # --- Tie rod / steering ---
    tierod_inner: Vec3 = (120.0, 320.0, 165.0)  # rack end (chassis) = R
    tierod_outer: Vec3 = (130.0, 715.0, 150.0)  # outer ball joint (knuckle) = P2

    # --- Wheel ---
    wheel_center: Vec3 = (0.0, 760.0, 300.0)
    wheel_radius: float = 300.0

    # --- Spring / damper (for motion-ratio / visual reference only) ---
    spring_top: Vec3 = (-20.0, 590.0, 680.0)
    spring_bottom: Vec3 = (10.0, 700.0, 400.0)

    # --- Optional full-vehicle parameters (for anti-dive estimate) ---
    wheelbase: float = 2600.0
    cg_height: float = 520.0
    static_camber_deg: float = -0.5
    static_toe_deg: float = 0.1

    def as_dict(self):
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.as_dict(), indent=2)

    @staticmethod
    def from_json(s: str) -> "Hardpoints":
        d = json.loads(s)
        valid = {f.name for f in fields(Hardpoints)}
        return Hardpoints(**{k: v for k, v in d.items() if k in valid})


def _v(p) -> np.ndarray:
    return np.array(p, dtype=float)


# --------------------------------------------------------------------------
# Static geometry (exact, no solver needed)
# --------------------------------------------------------------------------

def kingpin_axis(hp: Hardpoints):
    """Return (T, P1, unit direction T->P1)."""
    T = _v(hp.strut_top)
    P1 = _v(hp.lca_outer)
    d = P1 - T
    return T, P1, d / np.linalg.norm(d)


def static_angles(hp: Hardpoints) -> dict:
    """KPI, caster, scrub radius, trail, mechanical trail sign, static swing angle."""
    T, P1, d = kingpin_axis(hp)
    wc = _v(hp.wheel_center)

    # KPI: front-view (Y-Z) inclination from vertical
    kpi = np.degrees(np.arctan2(abs(d[1]), abs(d[2])))

    # Caster: side-view (X-Z) inclination from vertical
    caster = np.degrees(np.arctan2(abs(d[0]), abs(d[2])))

    # where the kingpin axis crosses the ground (Z = 0), extrapolating the line
    if abs(d[2]) < 1e-9:
        ground_pt = np.array([np.nan, np.nan, 0.0])
    else:
        t = (0.0 - T[2]) / d[2]
        ground_pt = T + t * d

    scrub_radius = wc[1] - ground_pt[1]   # +ve (classic): axis crosses ground inboard of contact patch center
    trail = ground_pt[0] - wc[0]          # +ve (classic mechanical trail): axis crosses ground ahead of contact patch

    return {
        "kpi_deg": kpi,
        "caster_deg": caster,
        "scrub_radius_mm": scrub_radius,
        "mechanical_trail_mm": trail,
        "kingpin_ground_point": ground_pt,
    }


def front_view_roll_center(hp: Hardpoints) -> dict:
    """Classic McPherson front-view roll-center construction (Y-Z plane)."""
    A = _v(hp.lca_front)[[1, 2]]
    B = _v(hp.lca_rear)[[1, 2]]
    lca_pivot = 0.5 * (A + B)                  # virtual inner pivot, front view
    P1 = _v(hp.lca_outer)[[1, 2]]
    T = _v(hp.strut_top)[[1, 2]]
    wc_y = _v(hp.wheel_center)[1]

    # Line A: LCA pivot -> P1
    dirA = P1 - lca_pivot
    # Line B: through T, perpendicular to strut axis (front view)
    strut_dir = P1 - T
    strut_dir = strut_dir if np.linalg.norm(strut_dir) > 1e-9 else np.array([0.0, 1.0])
    dirB = np.array([-strut_dir[1], strut_dir[0]])  # rotate 90 deg

    # Line B must pass through the STRUT TOP MOUNT (T), not P1: the strut
    # behaves like a slider pinned at T, so by instant-center theory the
    # knuckle's IC lies on the line through T perpendicular to the sliding
    # (strut) direction. (A common hand-drawing slip is to anchor this line
    # at P1 instead -- that collapses the construction onto P1 and is wrong.)
    ic = _line_intersect_2d(lca_pivot, dirA, T, dirB)

    cp = np.array([wc_y, 0.0])                 # tire contact patch, front view
    dirC = ic - cp
    # Roll centre = where line CP->IC crosses Y = 0 (vehicle centreline)
    if abs(dirC[0]) < 1e-9:
        rc = np.array([0.0, np.nan])
    else:
        t = (0.0 - cp[0]) / dirC[0]
        rc = cp + t * dirC

    return {
        "instant_center_yz": ic,
        "contact_patch_yz": cp,
        "roll_center_height_mm": rc[1],
        "roll_center_yz": rc,
    }


def _line_intersect_2d(p1, d1, p2, d2):
    """Intersection of line p1+t*d1 and p2+s*d2 in 2D."""
    A = np.array([d1, -d2]).T
    b = p2 - p1
    try:
        ts = np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        return np.array([np.nan, np.nan])
    return p1 + ts[0] * d1


def anti_dive_estimate(hp: Hardpoints) -> dict:
    """Simplified side-view swing-arm angle & anti-dive % (front axle, braking)."""
    A = _v(hp.lca_front)[[0, 2]]
    B = _v(hp.lca_rear)[[0, 2]]
    pivot = 0.5 * (A + B)
    P1 = _v(hp.lca_outer)[[0, 2]]
    v = P1 - pivot
    angle = np.degrees(np.arctan2(v[1], v[0]))  # from horizontal, side view
    pct_antidive = 100.0 * np.tan(np.radians(angle)) * hp.wheelbase / hp.cg_height
    return {"side_view_swing_angle_deg": angle, "anti_dive_pct_estimate": pct_antidive}


# --------------------------------------------------------------------------
# Kinematic sweep (numeric rigid-body solve)
# --------------------------------------------------------------------------

def _lca_circle_frame(hp: Hardpoints):
    A = _v(hp.lca_front)
    B = _v(hp.lca_rear)
    P1_0 = _v(hp.lca_outer)
    axis = B - A
    axis = axis / np.linalg.norm(axis)
    ap = P1_0 - A
    perp = ap - np.dot(ap, axis) * axis
    radius = np.linalg.norm(perp)
    e1 = perp / radius if radius > 1e-9 else np.array([0, 0, 1.0])
    e2 = np.cross(axis, e1)
    center = A + np.dot(ap, axis) * axis
    return center, e1, e2, radius


def _residuals(rv, P1_cur, T, R_static_data, tierod_inner, tierod_len):
    R = Rotation.from_rotvec(rv).as_matrix()
    S_cur = P1_cur + R @ R_static_data["S_rel"]
    d_cur = R @ R_static_data["strut_dir0"]
    r1 = np.cross(T - S_cur, d_cur)
    P2_cur = P1_cur + R @ R_static_data["P2_rel"]
    r2 = np.linalg.norm(P2_cur - tierod_inner) - tierod_len
    return np.concatenate([r1, [r2]])


def sweep(hp: Hardpoints, theta_deg_range=12.0, n_steps=41) -> dict:
    """Sweep the LCA through +/- theta_deg_range (deg) about its static
    position, solving the knuckle orientation at each step. Returns arrays
    of wheel travel (mm, +ve = jounce/up), camber (deg), toe (deg, +ve =
    toe-in), and roll-center height (mm) -- plus the raw knuckle transforms
    for 3-D plotting.
    """
    P1_0 = _v(hp.lca_outer)
    T = _v(hp.strut_top)
    S_0 = _v(hp.strut_knuckle)
    P2_0 = _v(hp.tierod_outer)
    R_inner = _v(hp.tierod_inner)
    wc_0 = _v(hp.wheel_center)

    tierod_len = np.linalg.norm(P2_0 - R_inner)
    strut_dir0 = (T - S_0)
    strut_dir0 = strut_dir0 / np.linalg.norm(strut_dir0)

    static_data = {
        "S_rel": S_0 - P1_0,
        "P2_rel": P2_0 - P1_0,
        "wc_rel": wc_0 - P1_0,
        "strut_dir0": strut_dir0,
    }

    center, e1, e2, radius = _lca_circle_frame(hp)
    # angle of the static P1 position on the circle
    v0 = P1_0 - center
    theta0 = np.arctan2(np.dot(v0, e2), np.dot(v0, e1))

    thetas = np.radians(np.linspace(-theta_deg_range, theta_deg_range, n_steps)) + theta0

    rv_guess = np.zeros(3)
    results = {
        "wheel_travel_mm": [],
        "camber_deg": [],
        "toe_deg": [],
        "roll_center_height_mm": [],
        "P1": [], "S": [], "P2": [], "wheel_center": [], "R": [],
    }

    static_wc_z = wc_0[2]

    for th in thetas:
        P1_cur = center + radius * (np.cos(th) * e1 + np.sin(th) * e2)
        # method='lm' (Levenberg-Marquardt) with tight tolerances: the
        # default 'trf' solver was observed to falsely "converge" and get
        # stuck at the previous (warm-started) solution for a few steps
        # near rv=0, silently freezing camber/toe there. 'lm' + tight
        # tolerances reliably drives the residual to ~machine precision at
        # every step.
        sol = least_squares(
            _residuals, rv_guess,
            args=(P1_cur, T, static_data, R_inner, tierod_len),
            method="lm", xtol=1e-15, ftol=1e-15, gtol=1e-15, max_nfev=5000,
        )
        if np.linalg.norm(sol.fun) > 1e-6:
            raise RuntimeError(
                f"Kinematic solver failed to converge at theta={np.degrees(th):.3f} deg "
                f"(residual norm={np.linalg.norm(sol.fun):.3e}). Hardpoints may be "
                f"geometrically inconsistent or the sweep range too large."
            )
        rv_guess = sol.x
        Rm = Rotation.from_rotvec(sol.x).as_matrix()

        S_cur = P1_cur + Rm @ static_data["S_rel"]
        P2_cur = P1_cur + Rm @ static_data["P2_rel"]
        wc_cur = P1_cur + Rm @ static_data["wc_rel"]

        euler = Rotation.from_matrix(Rm).as_euler("xyz", degrees=True)
        camber = hp.static_camber_deg + euler[0]
        toe = hp.static_toe_deg + euler[2]

        # roll-center height at this travel step: re-derive front-view
        # construction using the *current* P1/T front-view projections but
        # keeping the LCA-inner and strut-top chassis points fixed.
        hp_step = Hardpoints(**hp.as_dict())
        hp_step.lca_outer = tuple(P1_cur)
        hp_step.wheel_center = tuple(wc_cur)
        rc = front_view_roll_center(hp_step)

        results["wheel_travel_mm"].append(wc_cur[2] - static_wc_z)
        results["camber_deg"].append(camber)
        results["toe_deg"].append(toe)
        results["roll_center_height_mm"].append(rc["roll_center_height_mm"])
        results["P1"].append(P1_cur)
        results["S"].append(S_cur)
        results["P2"].append(P2_cur)
        results["wheel_center"].append(wc_cur)
        results["R"].append(Rm)

    for k in ["wheel_travel_mm", "camber_deg", "toe_deg", "roll_center_height_mm"]:
        results[k] = np.array(results[k])
    order = np.argsort(results["wheel_travel_mm"])
    for k in ["wheel_travel_mm", "camber_deg", "toe_deg", "roll_center_height_mm"]:
        results[k] = results[k][order]
    for k in ["P1", "S", "P2", "wheel_center", "R"]:
        results[k] = [results[k][i] for i in order]

    return results


if __name__ == "__main__":
    hp = Hardpoints()
    print("Static angles:", static_angles(hp))
    print("Roll center:", front_view_roll_center(hp))
    print("Anti-dive:", anti_dive_estimate(hp))
    res = sweep(hp)
    print("Travel range:", res["wheel_travel_mm"].min(), res["wheel_travel_mm"].max())
    print("Camber range:", res["camber_deg"].min(), res["camber_deg"].max())
    print("Toe range:", res["toe_deg"].min(), res["toe_deg"].max())
