"""
Tasit Statik Geometri Hesaplayici
----------------------------------
McPherson / Double Wishbone / Multi-link suspansiyon tiplerini destekleyen,
Ceyrek Tasit / Yarim Tasit / Tam Tasit sekmeli statik geometri hesaplayicisi.

Calistirmak icin:
    pip install -r requirements.txt
    streamlit run app.py
"""

from dataclasses import fields, replace

import matplotlib.pyplot as plt
import streamlit as st

import kinematics as K
import plotting as P


def show_fig(fig):
    """st.pyplot() then close -- prevents matplotlib's global figure
    registry from growing unbounded across Streamlit reruns."""
    st.pyplot(fig)
    plt.close(fig)

st.set_page_config(page_title="Tasit Statik Geometri Hesaplayici", layout="wide")

TYPE_KEYS = list(K.SUSPENSION_TYPES.keys())

FIELD_LABELS = {
    "wheel_center": "Tekerlek merkezi",
    "contact_patch": "Lastik temas noktasi",
    "tierod_inner": "Rot kolu ic (kremayer / sase)",
    "tierod_outer": "Rot kolu dis (knuckle)",
    "wheel_radius": "Tekerlek yaricapi (mm)",
    "static_camber_deg": "Statik camber (deg, - = negatif camber)",
    "static_toe_deg": "Statik toe (deg, + = toe-in)",
    "lca_front": "LCA ic pivot - on (sase)",
    "lca_rear": "LCA ic pivot - arka (sase)",
    "lca_outer": "LCA dis rot bilyesi P1 (knuckle)",
    "top_mount": "Strut ust montaj T (sase)",
    "strut_lwr_mount": "Strut alt montaj S (knuckle, gorsel referans)",
    "uca_front": "UCA ic pivot - on (sase)",
    "uca_rear": "UCA ic pivot - arka (sase)",
    "uca_outer": "UCA dis top mafsal (knuckle)",
    "upper_control_link_chassis": "Upper control link - sase",
    "upper_control_link_knuckle": "Upper control link - knuckle",
    "lower_control_link_chassis": "Lower control link - sase",
    "lower_control_link_knuckle": "Lower control link - knuckle",
    "toe_link_chassis": "Toe link - sase",
    "toe_link_knuckle": "Toe link - knuckle",
    "trailing_link_chassis": "Trailing link - sase",
    "trailing_link_knuckle": "Trailing link - knuckle",
    "camber_link_chassis": "Camber link - sase",
    "camber_link_knuckle": "Camber link - knuckle",
}

_BASE_FIELD_NAMES = {f.name for f in fields(K.Suspension)}


def point_input(label, key, default):
    # bare st.xxx (not st.sidebar.xxx / re-qualified) so this nests correctly
    # inside whichever `with ...expander(...)` block it's called from.
    st.markdown(f"**{label}**")
    c1, c2, c3 = st.columns(3)
    x = c1.number_input("X", value=float(default[0]), key=f"{key}_x", step=5.0, format="%.1f")
    y = c2.number_input("Y", value=float(default[1]), key=f"{key}_y", step=5.0, format="%.1f")
    z = c3.number_input("Z", value=float(default[2]), key=f"{key}_z", step=5.0, format="%.1f")
    return (x, y, z)


def _render_field(f, key_prefix):
    default = f.default
    label = FIELD_LABELS.get(f.name, f.name)
    if isinstance(default, tuple):
        return point_input(label, f"{key_prefix}_{f.name}", default)
    return st.number_input(label, value=float(default), step=1.0, key=f"{key_prefix}_{f.name}")


def hardpoint_form(cls, key_prefix):
    """Secilen suspansiyon tipine gore hardpoint alanlarini dinamik olarak
    cizer; alan isimleri kinematics.py'deki literatur standardini kullanir."""
    kwargs = {}
    # NOTE: iterate fields(cls) (not fields(K.Suspension)) so that a
    # subclass's OWN overridden defaults (e.g. McPherson/DoubleWishbone's
    # real wheel_center/contact_patch/tierod_* coordinates) are used --
    # fields(K.Suspension) would always return the base class's generic
    # placeholder defaults regardless of what the subclass redeclared.
    cls_fields = [f for f in fields(cls) if f.name not in ("side", "axle")]
    base_fields = [f for f in cls_fields if f.name in _BASE_FIELD_NAMES]
    extra_fields = [f for f in cls_fields if f.name not in _BASE_FIELD_NAMES]

    with st.expander("Genel Noktalar", expanded=True):
        for f in base_fields:
            kwargs[f.name] = _render_field(f, key_prefix)

    if extra_fields:
        type_label = K.SUSPENSION_TYPE_LABELS_TR[cls.__name__]
        with st.expander(f"{type_label} Noktalari", expanded=True):
            for f in extra_fields:
                kwargs[f.name] = _render_field(f, key_prefix)

    return kwargs


def corner_selector(key_prefix, default_type="McPherson", default_axle="front",
                     default_side="R", show_axle=True, show_side=True):
    """Tip (+ opsiyonel konum) secicileri ve dinamik hardpoint formunu
    cizer; olusan Suspension alt sinifi ornegini dondurur."""
    type_name = st.selectbox(
        "Suspansiyon Tipi", TYPE_KEYS,
        format_func=lambda k: K.SUSPENSION_TYPE_LABELS_TR[k],
        index=TYPE_KEYS.index(default_type), key=f"{key_prefix}_type",
    )
    cls = K.SUSPENSION_TYPES[type_name]

    axle, side = default_axle, default_side
    if show_axle or show_side:
        cols = st.columns(2)
        if show_axle:
            axle = cols[0].selectbox(
                "Konum (aks)", ["front", "rear"],
                format_func=lambda a: "On" if a == "front" else "Arka",
                index=0 if default_axle == "front" else 1, key=f"{key_prefix}_axle",
            )
        if show_side:
            side = cols[1].selectbox(
                "Taraf", ["R", "L"], format_func=lambda s: "Sag" if s == "R" else "Sol",
                index=0 if default_side == "R" else 1, key=f"{key_prefix}_side",
            )

    kwargs = hardpoint_form(cls, key_prefix)
    return cls(side=side, axle=axle, **kwargs)


def safe_call(fn, *args, **kwargs):
    """fn'yi calistirir; K.GeometryError (dejenere/cakisik hardpoint gibi
    veri hatalari) ya da beklenmeyen baska bir hata olursa st.error ile
    acikca gosterip None dondurur -- sayfanin tamamen cokmesini onler."""
    try:
        return fn(*args, **kwargs)
    except K.GeometryError as e:
        st.error(f"Geometri hatasi: {e}")
        return None
    except Exception as e:
        st.error(f"Beklenmeyen hata: {e}")
        return None


def show_static_metrics(corner: K.Suspension):
    ang = safe_call(corner.static_angles)
    if ang is None:
        return
    c1, c2, c3, c4, c5, c6 = st.columns(6)
    c1.metric("KPI", f"{ang['kpi_deg']:.2f}°")
    c2.metric("Caster", f"{ang['caster_deg']:.2f}°")
    c3.metric("Camber (statik)", f"{ang['static_camber_deg']:.2f}°")
    c4.metric("Toe (statik)", f"{ang['static_toe_deg']:.2f}°")
    c5.metric("Scrub Radius", f"{ang['scrub_radius_mm']:.1f} mm")
    c6.metric("Mekanik Trail", f"{ang['mechanical_trail_mm']:.1f} mm")
    if not ang["kingpin_axis_exact"]:
        st.warning(
            "Bu suspansiyon tipi icin kingpin ekseni (ve dolayisiyla yukaridaki "
            "degerler) YAKLASIKTIR -- asagidaki notlara bakin."
        )
    for note in corner.geometry_notes():
        st.caption("ℹ️ " + note)


def load_json_into_widgets(key_prefix, json_text):
    loaded = K.Suspension.from_json(json_text)
    st.session_state[f"{key_prefix}_type"] = type(loaded).__name__
    st.session_state[f"{key_prefix}_axle"] = loaded.axle
    st.session_state[f"{key_prefix}_side"] = loaded.side
    for f in fields(type(loaded)):
        if f.name in ("side", "axle"):
            continue
        val = getattr(loaded, f.name)
        wkey = f"{key_prefix}_{f.name}"
        if isinstance(val, (tuple, list)):
            st.session_state[f"{wkey}_x"] = float(val[0])
            st.session_state[f"{wkey}_y"] = float(val[1])
            st.session_state[f"{wkey}_z"] = float(val[2])
        else:
            st.session_state[wkey] = float(val)


st.title("Tasit Statik Geometri Hesaplayici")
st.caption(
    "McPherson / Double Wishbone / Multi-link icin statik hardpoint "
    "koordinatlarindan KPI, caster, scrub radius, mekanik trail, roll "
    "center, wheelbase, roll axis ve anti-dive/squat hesaplar. Sadece "
    "STATIK geometri -- bump/rebound tarama analizi bu surumde yok."
)

tab_quarter, tab_half, tab_full = st.tabs(["Ceyrek Tasit", "Yarim Tasit", "Tam Tasit"])


# ============================================================================
# SEKME 1 -- Ceyrek Tasit
# ============================================================================

with tab_quarter:
    st.header("Ceyrek Tasit (Quarter Car)")

    up = st.file_uploader("Hardpoint JSON yukle (opsiyonel)", type=["json"], key="qc_upload")
    if up is not None:
        # NOTE: st.rerun() must NOT be inside this try block -- it raises an
        # internal Streamlit control-flow exception that a broad `except
        # Exception` would silently swallow, breaking the rerun.
        load_ok = False
        try:
            load_json_into_widgets("qc", up.read().decode("utf-8"))
            load_ok = True
        except Exception as e:
            st.error(f"JSON okunamadi: {e}")
        if load_ok:
            st.success("Yuklendi.")
            st.rerun()

    corner_qc = corner_selector("qc")

    st.subheader("Statik Geometri Ciktilari")
    show_static_metrics(corner_qc)

    st.download_button(
        "Bu kosenin hardpoint'lerini JSON olarak indir",
        data=corner_qc.to_json(),
        file_name=f"{type(corner_qc).__name__.lower()}_{corner_qc.axle}_{corner_qc.side.lower()}.json",
        mime="application/json",
    )

    col3d, colrc = st.columns(2)
    with col3d:
        fig3d = safe_call(P.plot_3d_linkage, corner_qc)
        if fig3d is not None:
            show_fig(fig3d)
    with colrc:
        figrc = safe_call(P.plot_front_view_roll_center, corner_qc)
        if figrc is not None:
            show_fig(figrc)


# ============================================================================
# SEKME 2 -- Yarim Tasit
# ============================================================================

with tab_half:
    st.header("Yarim Tasit (Half Car -- On + Arka)")
    st.caption("Klasik 'bicycle model': tek bir on ve tek bir arka kose uzerinden.")

    colf, colr = st.columns(2)
    with colf:
        st.subheader("On Suspansiyon")
        front_hc = corner_selector("hc_front", default_type="McPherson", default_axle="front",
                                    show_axle=False, show_side=False)
    with colr:
        st.subheader("Arka Suspansiyon")
        rear_hc = corner_selector("hc_rear", default_type="MultiLink", default_axle="rear",
                                   show_axle=False, show_side=False)

    st.subheader("Yarim Tasit Ciktilari")
    track_f = safe_call(K.track_width_mm, front_hc)
    track_r = safe_call(K.track_width_mm, rear_hc)
    rc_f_data = safe_call(front_hc.front_view_roll_center)
    rc_r_data = safe_call(rear_hc.front_view_roll_center)

    c1, c2, c3, c4 = st.columns(4)
    c1.metric("On Iz Genisligi (Track Width)", f"{track_f:.0f} mm" if track_f is not None else "N/A")
    c2.metric("Arka Iz Genisligi (Track Width)", f"{track_r:.0f} mm" if track_r is not None else "N/A")
    c3.metric("On Roll Center Yuksekligi",
              f"{rc_f_data['roll_center_height_mm']:.1f} mm" if rc_f_data is not None else "N/A")
    c4.metric("Arka Roll Center Yuksekligi",
              f"{rc_r_data['roll_center_height_mm']:.1f} mm" if rc_r_data is not None else "N/A")

    col3d, colrc = st.columns(2)
    with col3d:
        fig_f = safe_call(P.plot_front_view_roll_center, front_hc)
        if fig_f is not None:
            show_fig(fig_f)
            st.caption("On roll-center insaasi")
    with colrc:
        fig_r = safe_call(P.plot_front_view_roll_center, rear_hc)
        if fig_r is not None:
            show_fig(fig_r)
            st.caption("Arka roll-center insaasi")


# ============================================================================
# SEKME 3 -- Tam Tasit
# ============================================================================

with tab_full:
    st.header("Tam Tasit (Full Car -- 4 Kose)")

    mirror = st.checkbox(
        "Sol tarafi sag taraftan otomatik aynala (simetrik arac icin onerilen)",
        value=True, key="fc_mirror",
    )
    cg_height = st.number_input("Agirlik merkezi yuksekligi (mm)", value=520.0, step=10.0, key="fc_cg")

    st.subheader("On Aks")
    colfr, colfl = st.columns(2)
    with colfr:
        st.markdown("**On Sag**")
        fr = corner_selector("fc_fr", default_type="McPherson", default_axle="front",
                              default_side="R", show_axle=False, show_side=False)
    with colfl:
        st.markdown("**On Sol**")
        if mirror:
            fl = replace(fr, side="L")
            st.info("On sag'dan aynalandi.")
        else:
            fl = corner_selector("fc_fl", default_type="McPherson", default_axle="front",
                                  default_side="L", show_axle=False, show_side=False)

    st.subheader("Arka Aks")
    colrr, colrl = st.columns(2)
    with colrr:
        st.markdown("**Arka Sag**")
        rr = corner_selector("fc_rr", default_type="MultiLink", default_axle="rear",
                              default_side="R", show_axle=False, show_side=False)
    with colrl:
        st.markdown("**Arka Sol**")
        if mirror:
            rl = replace(rr, side="L")
            st.info("Arka sag'dan aynalandi.")
        else:
            rl = corner_selector("fc_rl", default_type="MultiLink", default_axle="rear",
                                  default_side="L", show_axle=False, show_side=False)

    st.subheader("Tam Tasit Ciktilari")
    wb = safe_call(K.wheelbase_mm, fr, rr)
    axis = safe_call(K.roll_axis, fr, rr)
    wb_safe = wb if (wb is not None and wb != 0) else 1.0
    ad = safe_call(fr.anti_geometry, wb_safe, cg_height)
    asq = safe_call(rr.anti_geometry, wb_safe, cg_height)

    def _fmt_anti(d, key):
        """Guvenilirlik (reliable) bayragini dikkate alarak anti-dive/squat
        yuzdesini bicimlendirir; astronomik/NaN degerler yerine acik bir
        'N/A' gosterir."""
        if d is None:
            return "N/A"
        if not d.get("reliable", True):
            return "N/A (guvenilmez: swing acisi ±90°'ye cok yakin)"
        val = d.get(key, float("nan"))
        if val != val:  # NaN kontrolu
            return "N/A"
        return f"{val:.1f} %"

    c1, c2, c3, c4 = st.columns(4)
    c1.metric("Dingil Mesafesi (Wheelbase)", f"{wb:.0f} mm" if wb is not None else "N/A")
    c2.metric("Roll Axis Acisi", f"{axis['roll_axis_angle_deg']:.2f}°" if axis is not None else "N/A")
    c3.metric("Anti-dive (on, %)", _fmt_anti(ad, "anti_dive_pct"))
    c4.metric("Anti-squat (arka, %)", _fmt_anti(asq, "anti_squat_pct"))

    st.caption(
        "Roll axis acisi = on ve arka roll-center yuksekligi farkinin dingil "
        "mesafesine gore side-view egimi. Anti-dive/squat, her koseninin "
        "kendi side-view swing-arm/instant-center insaasindan (bkz. "
        "kinematics.py) turetilir; agirlik merkezi yuksekligi yukarida "
        "girilen degerdir."
    )

    st.subheader("Tum Koseler -- Ozet")
    corners = {"On Sag": fr, "On Sol": fl, "Arka Sag": rr, "Arka Sol": rl}
    header = "| Kose | Tip | KPI | Caster | Camber | Toe | Scrub | Trail |\n"
    header += "|---|---|---|---|---|---|---|---|\n"
    rows = ""
    for label, c in corners.items():
        a = safe_call(c.static_angles)
        if a is None:
            rows += f"| {label} | {type(c).__name__} | - | - | - | - | - | - |\n"
            continue
        rows += (
            f"| {label} | {type(c).__name__} | {a['kpi_deg']:.1f}° | {a['caster_deg']:.1f}° | "
            f"{a['static_camber_deg']:.1f}° | {a['static_toe_deg']:.1f}° | "
            f"{a['scrub_radius_mm']:.0f} mm | {a['mechanical_trail_mm']:.0f} mm |\n"
        )
    st.markdown(header + rows)
