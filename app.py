"""
McPherson Süspansiyon Hardpoint Görselleştirici
------------------------------------------------
Hardpoint koordinatlarini gir; KPI, caster, scrub radius, trail, roll
center, anti-dive gibi statik degerleri ve camber/toe/roll-center'in
tekerlek hareketine (bump/rebound) gore degisim egrilerini hesaplar ve
cizer.

Calistirmak icin:
    pip install -r requirements.txt
    streamlit run app.py

Metodoloji ve varsayimlar icin kinematics.py'nin docstring'ine ve
README.md'ye bakin.
"""

from dataclasses import fields

import streamlit as st

import kinematics as K
import plotting as P

st.set_page_config(page_title="McPherson Hardpoint Analizi", layout="wide")


# --------------------------------------------------------------------------
# Session state / hardpoint yonetimi
# --------------------------------------------------------------------------

if "hp" not in st.session_state:
    st.session_state.hp = K.Hardpoints()


def point_input(label, key, default):
    # NOTE: use the bare `st.xxx` calls here (not `st.sidebar.xxx`) so this
    # renders correctly *nested inside* whichever `with st.sidebar.expander(...)`
    # block it's called from -- `st.sidebar.xxx` always targets the sidebar's
    # top level and would render outside/after the expander instead.
    st.markdown(f"**{label}**")
    c1, c2, c3 = st.columns(3)
    x = c1.number_input("X", value=float(default[0]), key=f"{key}_x", step=5.0, format="%.1f")
    y = c2.number_input("Y", value=float(default[1]), key=f"{key}_y", step=5.0, format="%.1f")
    z = c3.number_input("Z", value=float(default[2]), key=f"{key}_z", step=5.0, format="%.1f")
    return (x, y, z)


st.sidebar.title("Hardpoint Girisi")
st.sidebar.caption(
    "Koordinat sistemi: X ileri (+), Y disa/tekerlege dogru (+), "
    "Z yukari (+), birim mm. Tek bir kose (orn. on-sag) icin girin."
)

uploaded = st.sidebar.file_uploader("Hardpoint JSON yukle", type=["json"])
if uploaded is not None:
    try:
        st.session_state.hp = K.Hardpoints.from_json(uploaded.read().decode("utf-8"))
        st.sidebar.success("Hardpoint'ler yuklendi.")
    except Exception as e:
        st.sidebar.error(f"JSON okunamadi: {e}")

if st.sidebar.button("Ornek degerlere sifirla"):
    st.session_state.hp = K.Hardpoints()
    for f in fields(K.Hardpoints):
        for suffix in ("_x", "_y", "_z"):
            st.session_state.pop(f"{f.name}{suffix}", None)
    st.rerun()

hp0 = st.session_state.hp

with st.sidebar.expander("LCA (Alt Salincak)", expanded=True):
    lca_front = point_input("Ic pivot - on (chassis)", "lca_front", hp0.lca_front)
    lca_rear = point_input("Ic pivot - arka (chassis)", "lca_rear", hp0.lca_rear)
    lca_outer = point_input("Dis rot bilyesi P1 (knuckle)", "lca_outer", hp0.lca_outer)

with st.sidebar.expander("Strut", expanded=False):
    strut_top = point_input("Ust montaj T (chassis)", "strut_top", hp0.strut_top)
    strut_knuckle = point_input("Knuckle referans S (kinematik cozucu icin)", "strut_knuckle", hp0.strut_knuckle)

with st.sidebar.expander("Rot Kolu / Direksiyon", expanded=False):
    tierod_inner = point_input("Kremayer ucu R (chassis)", "tierod_inner", hp0.tierod_inner)
    tierod_outer = point_input("Dis rot bilyesi P2 (knuckle)", "tierod_outer", hp0.tierod_outer)

with st.sidebar.expander("Tekerlek", expanded=False):
    wheel_center = point_input("Tekerlek merkezi", "wheel_center", hp0.wheel_center)
    wheel_radius = st.number_input("Tekerlek yaricapi (mm)", value=float(hp0.wheel_radius), step=5.0)

with st.sidebar.expander("Yay / Damper (sadece referans)", expanded=False):
    spring_top = point_input("Yay ust", "spring_top", hp0.spring_top)
    spring_bottom = point_input("Yay alt", "spring_bottom", hp0.spring_bottom)

with st.sidebar.expander("Arac parametreleri (anti-dive tahmini icin)", expanded=False):
    wheelbase = st.number_input("Dingil mesafesi (mm)", value=float(hp0.wheelbase), step=10.0)
    cg_height = st.number_input("Agirlik merkezi yuksekligi (mm)", value=float(hp0.cg_height), step=10.0)
    static_camber_deg = st.number_input("Statik camber (deg)", value=float(hp0.static_camber_deg), step=0.1)
    static_toe_deg = st.number_input("Statik toe (deg, + = toe-in)", value=float(hp0.static_toe_deg), step=0.05)

hp = K.Hardpoints(
    lca_front=lca_front, lca_rear=lca_rear, lca_outer=lca_outer,
    strut_top=strut_top, strut_knuckle=strut_knuckle,
    tierod_inner=tierod_inner, tierod_outer=tierod_outer,
    wheel_center=wheel_center, wheel_radius=wheel_radius,
    spring_top=spring_top, spring_bottom=spring_bottom,
    wheelbase=wheelbase, cg_height=cg_height,
    static_camber_deg=static_camber_deg, static_toe_deg=static_toe_deg,
)
st.session_state.hp = hp

st.sidebar.download_button(
    "Hardpoint'leri JSON olarak indir",
    data=hp.to_json(),
    file_name="mcpherson_hardpoints.json",
    mime="application/json",
)

st.sidebar.divider()
st.sidebar.subheader("Sweep (bump/rebound)")
theta_range = st.sidebar.slider("LCA acisi tarama araligi (+/- derece)", 2.0, 25.0, 12.0, 0.5)
n_steps = st.sidebar.slider("Adim sayisi", 11, 81, 41, 2)


# --------------------------------------------------------------------------
# Ana panel
# --------------------------------------------------------------------------

st.title("McPherson Süspansiyon — Hardpoint Analizi")
st.caption(
    "Sol menuden hardpoint koordinatlarini girin. Tum sonuclar bu noktalardan "
    "geometrik olarak turetilir (bkz. 'Metodoloji' sekmesi)."
)

ang = K.static_angles(hp)
rc = K.front_view_roll_center(hp)
ad = K.anti_dive_estimate(hp)

c1, c2, c3, c4, c5, c6 = st.columns(6)
c1.metric("KPI", f"{ang['kpi_deg']:.2f}°")
c2.metric("Caster", f"{ang['caster_deg']:.2f}°")
c3.metric("Scrub Radius", f"{ang['scrub_radius_mm']:.1f} mm")
c4.metric("Mekanik Trail", f"{ang['mechanical_trail_mm']:.1f} mm")
c5.metric("Roll Center (statik)", f"{rc['roll_center_height_mm']:.1f} mm")
c6.metric("Anti-dive (tahmini)", f"{ad['anti_dive_pct_estimate']:.1f} %")

tab_3d, tab_rc, tab_curves, tab_rc_travel, tab_meta = st.tabs(
    ["3D Gorunum", "Roll Center Insaasi", "Camber / Toe Egrileri", "Roll Center vs Travel", "Metodoloji & JSON"]
)

with tab_3d:
    st.pyplot(P.plot_3d_linkage(hp))

with tab_rc:
    st.pyplot(P.plot_front_view_roll_center(hp))
    st.caption(
        "Klasik McPherson on-gorunum roll-center insaasi: Line A = LCA hattinin "
        "P1 uzerinden uzatilmasi; Line B = T (strut ust montaj) noktasindan "
        "strut eksenine dik cizilen hat; IC = A ile B'nin kesisimi; roll center = "
        "lastik temas noktasindan IC'ye cizilen hattin arac merkez hattini kestigi nokta."
    )

with tab_curves:
    try:
        res = K.sweep(hp, theta_deg_range=theta_range, n_steps=int(n_steps))
        st.pyplot(P.plot_camber_toe(res))
        st.info(
            f"Tarama sonucu tekerlek hareketi araligi: "
            f"{res['wheel_travel_mm'].min():.1f} mm ile {res['wheel_travel_mm'].max():.1f} mm arasi "
            f"(+ = bump / sikisma, - = rebound / uzama)."
        )
    except RuntimeError as e:
        st.error(f"Kinematik cozucu yakinsamadi: {e}")

with tab_rc_travel:
    try:
        res = K.sweep(hp, theta_deg_range=theta_range, n_steps=int(n_steps))
        st.pyplot(P.plot_roll_center_travel(res))
    except RuntimeError as e:
        st.error(f"Kinematik cozucu yakinsamadi: {e}")

with tab_meta:
    st.text(K.__doc__)
    st.subheader("Guncel hardpoint JSON")
    st.code(hp.to_json(), language="json")
