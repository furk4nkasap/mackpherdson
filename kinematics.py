"""
Tasit Statik Geometri Hesaplayici - kinematics.py
--------------------------------------------------
Cok tipli (McPherson / Double Wishbone / Multi-link) statik suspansiyon
geometrisi hesabi. Sadece STATIK hardpoint koordinatlarindan turetilen
geometrik ciktilari hesaplar; dinamik bump/rebound taramasi bu surumde YOK
(bilerek kaldirildi -- OOP mimarisi kurulduktan sonra ayri bir katman olarak
geri eklenebilir).

Mimari
------
`Suspension` taban sinifi ortak alanlari (wheel_center, contact_patch,
tierod_inner, tierod_outer, wheel_radius, statik camber/toe, side, axle) ve
ortak hesap metodlarini (static_angles, front_view_roll_center,
anti_geometry) tasir. Alt siniflar (McPherson, DoubleWishbone, MultiLink)
kendi hardpoint alanlarini ekler ve su "template method" kancalarini
doldurur:

    kingpin_axis()      -> (T, P1) 3D noktalari; eksen = P1 - T
    front_view_lines()  -> ((pA,dA),(pB,dB)) Y-Z duzleminde 2 dogru
                            (front-view instant center insaasi icin)
    side_view_lines()   -> ((pA,dA),(pB,dB)) X-Z duzleminde 2 dogru
                            (anti-dive/squat insaasi icin)
    plot_points()        -> {etiket: 3D nokta} (gorsellestirme icin)
    plot_edges()          -> [(p1,p2,renk,stil,etiket), ...] (gorsellestirme)
    geometry_notes()      -> [str, ...] yontem/yaklaşıklık notlari

Taban sinif bu kancalari kullanarak KPI/caster/scrub/trail, front-view roll
center ve anti-dive/squat hesaplarini TEK YERDE (DRY) yapar.

Koordinat sistemi: X ileri (+), Y disa/tekerlege dogru SAG taraf icin (+),
Z yukari (+), birim mm. Hardpoint'ler HER ZAMAN "sag taraf" (outboard = +Y)
kabulune gore girilir; `side='L'` secildiginde Y isaretleri hesap sirasinda
otomatik aynalanir (bkz. `_p()`), saklanan ham degerler degismez.

Camber ve toe UYARISI: bu degerler kontrol kolu/link hardpoint'lerinden
turetilemez (knuckle uzerindeki tekerlek montaj yuzeyinin acisi, ball-joint
konumlarindan bagimsiz bir tasarim/imalat degeridir). Bu yuzden
`static_camber_deg` / `static_toe_deg` dogrudan olculen/tasarlanan girdi
degerleridir, hesaplanan cikti degil -- ama yine de geometrinin bir parcasi
olarak raporlanir.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields
from typing import Tuple

import numpy as np

Vec3 = Tuple[float, float, float]


class GeometryError(ValueError):
    """Gecersiz/dejenere hardpoint geometrisi icin firlatilir (orn. cakisik
    noktalar -> sifir uzunluklu eksen/dogru). Bunlar veri hatasidir ve acik
    bir mesajla durdurulmalidir; A ile B dogrusunun birbirine PARALEL olmasi
    (IC tanimsiz) ise gecerli bir geometrik durumdur ve NaN olarak
    raporlanir, hata firlatilmaz -- ayrimi asagidaki fonksiyonlarda not
    edilmistir."""


# --------------------------------------------------------------------------
# Kucuk yardimcilar
# --------------------------------------------------------------------------

def _line_intersect_2d(p1, d1, p2, d2):
    """Iki 2D dogrunun (p1+t*d1) ve (p2+s*d2) kesisimi.

    d1/d2 sifir uzunluklu ise (iki hardpoint cakismis demektir) bu bir veri
    hatasidir -> GeometryError. d1 ile d2 birbirine paralel ise (IC sonsuzda,
    ornegin front-view'da iki kol tam paralelse) bu gecerli bir geometrik
    durumdur -> NaN dondurulur, hata firlatilmaz.
    """
    d1 = np.asarray(d1, dtype=float)
    d2 = np.asarray(d2, dtype=float)
    if np.linalg.norm(d1) < 1e-9 or np.linalg.norm(d2) < 1e-9:
        raise GeometryError(
            "Front/side-view dogru yonu sifir uzunlukta -- hardpoint "
            "noktalarindan ikisi cakismis olabilir. Koordinatlari kontrol edin."
        )
    A = np.array([d1, -d2]).T
    b = np.array(p2, dtype=float) - np.array(p1, dtype=float)
    try:
        ts = np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        return np.array([np.nan, np.nan])
    return np.array(p1, dtype=float) + ts[0] * d1


def _perp_2d(v):
    v = np.asarray(v, dtype=float)
    return np.array([-v[1], v[0]])


# --------------------------------------------------------------------------
# Taban sinif
# --------------------------------------------------------------------------

@dataclass
class Suspension:
    """Tum suspansiyon tipleri icin ortak alanlar ve hesap mantigi."""

    side: str = "R"        # 'R' (sag) veya 'L' (sol) -- Y isaretini aynalar
    axle: str = "front"    # 'front' veya 'rear' -- sadece etiketleme/agregasyon icin

    # --- Genel noktalar (tum tiplerde ortak) ---
    wheel_center: Vec3 = (0.0, 760.0, 300.0)
    contact_patch: Vec3 = (0.0, 760.0, 0.0)
    tierod_inner: Vec3 = (120.0, 320.0, 165.0)
    tierod_outer: Vec3 = (130.0, 715.0, 150.0)
    wheel_radius: float = 300.0

    # --- Dogrudan girilen (turetilemeyen) olcumler ---
    static_camber_deg: float = -0.5
    static_toe_deg: float = 0.1

    # Alt siniflar override eder: kingpin ekseni tam mi (2 top mafsalindan)
    # yoksa yaklasik mi (>2 linkli mekanizmadan sadelestirme ile)?
    KINGPIN_AXIS_EXACT = True

    # ---- Alt siniflarin doldurmasi gereken kancalar --------------------

    def kingpin_axis(self):
        """(T, P1) -- iki 3D nokta. Eksen yonu = normalize(P1 - T)."""
        raise NotImplementedError

    def front_view_lines(self):
        """((pA,dA),(pB,dB)) -- Y-Z duzleminde iki dogru (nokta, yon)."""
        raise NotImplementedError

    def side_view_lines(self):
        """((pA,dA),(pB,dB)) -- X-Z duzleminde iki dogru (nokta, yon)."""
        raise NotImplementedError

    def plot_points(self) -> dict:
        pts = {
            "Wheel center": self._p("wheel_center"),
            "Contact patch": self._p("contact_patch"),
            "Tie rod inner": self._p("tierod_inner"),
            "Tie rod outer": self._p("tierod_outer"),
        }
        return pts

    def plot_edges(self):
        T, P1 = self.kingpin_axis()
        ang = self.static_angles()
        ground_pt = ang["kingpin_ground_point"]  # already a full 3D (x,y,0) point
        edges = [
            (T, ground_pt, "tab:red", "--", "Kingpin axis (extended)"),
            (self._p("tierod_inner"), self._p("tierod_outer"), "tab:green", "-", "Tie rod"),
        ]
        return edges

    def geometry_notes(self):
        return [
            "Camber ve toe, kontrol kollari/linklerden turetilmez; dogrudan "
            "girilen (olculen/tasarlanan) degerlerdir."
        ]

    # ---- Yardimci: side='L' icin Y isaretini aynala ---------------------

    def _p(self, field_name: str) -> np.ndarray:
        v = np.array(getattr(self, field_name), dtype=float)
        if str(self.side).upper().startswith("L"):
            v = v * np.array([1.0, -1.0, 1.0])
        return v

    # ---- Ortak (taban sinifta tek yerde yazilan) hesaplar ---------------

    def static_angles(self) -> dict:
        """KPI, caster, scrub radius, mekanik trail -- kingpin ekseninden.

        Raises:
            GeometryError: kingpin ekseni sifir uzunluktaysa (T ve P1 ayni
                nokta) veya sonucta NaN/Inf olusursa -- bu bir veri
                hatasidir (hardpoint'ler yanlislikla cakismis olabilir).
        """
        T, P1 = self.kingpin_axis()
        diff = P1 - T
        norm = np.linalg.norm(diff)
        if not np.isfinite(norm) or norm < 1e-6:
            raise GeometryError(
                f"{type(self).__name__} ({self.axle}/{self.side}): kingpin "
                "ekseni sifir uzunlukta -- iki ucu (T ve P1) hemen hemen "
                "ayni noktada. Hardpoint koordinatlarini kontrol edin."
            )
        d = diff / norm
        cp = self._p("contact_patch")

        kpi = np.degrees(np.arctan2(abs(d[1]), abs(d[2])))
        caster = np.degrees(np.arctan2(abs(d[0]), abs(d[2])))

        # d normalize edilmis oldugu icin |d[2]| burada matematiksel olarak
        # her zaman <= 1'dir; eksen tam yatay ise (|d[2]| ~ 0, orn. T ve P1
        # ayni yukseklikte) yer kesisimi tanimsizdir -- bu GECERLI bir
        # geometrik durumdur (hata degil), NaN olarak raporlanir.
        if abs(d[2]) < 1e-9:
            ground_pt = np.array([np.nan, np.nan, 0.0])
        else:
            t = (0.0 - T[2]) / d[2]
            ground_pt = T + t * d

        # scrub radius must read the same (classic +ve = kingpin axis
        # crosses ground inboard of the contact patch) on both sides of a
        # symmetric vehicle. Since _p() mirrors Y for the left side,
        # "outboard" points in -Y there instead of +Y -- correct for that so
        # the sign reflects inboard/outboard, not raw +Y/-Y.
        outboard_sign = -1.0 if str(self.side).upper().startswith("L") else 1.0
        scrub_radius = outboard_sign * (cp[1] - ground_pt[1])
        mechanical_trail = ground_pt[0] - cp[0]

        if not np.isfinite([kpi, caster]).all():
            raise GeometryError(
                f"{type(self).__name__} ({self.axle}/{self.side}): KPI/caster "
                "hesaplamasi sonlu olmayan (NaN/Inf) bir deger uretti -- "
                "kingpin ekseni ve contact_patch noktalarini kontrol edin."
            )

        return {
            "kpi_deg": kpi,
            "caster_deg": caster,
            "scrub_radius_mm": scrub_radius,
            "mechanical_trail_mm": mechanical_trail,
            "kingpin_ground_point": ground_pt,
            "static_camber_deg": self.static_camber_deg,
            "static_toe_deg": self.static_toe_deg,
            "kingpin_axis_exact": self.KINGPIN_AXIS_EXACT,
        }

    def front_view_roll_center(self) -> dict:
        (pA, dA), (pB, dB) = self.front_view_lines()
        ic = _line_intersect_2d(pA, dA, pB, dB)
        cp = self._p("contact_patch")[[1, 2]]
        dirC = ic - cp
        if abs(dirC[0]) < 1e-9 or not np.isfinite(ic).all():
            rc = np.array([0.0, np.nan])
        else:
            t = (0.0 - cp[0]) / dirC[0]
            rc = cp + t * dirC
        return {
            "instant_center_yz": ic,
            "contact_patch_yz": cp,
            "roll_center_yz": rc,
            "roll_center_height_mm": rc[1],
        }

    def anti_geometry(self, wheelbase_mm: float, cg_height_mm: float) -> dict:
        """Basitlestirilmis side-view swing-arm / IC yontemiyle anti-dive
        (on aks, frenleme) veya anti-squat (arka aks, itme) tahmini.

        Raises:
            GeometryError: wheelbase_mm veya cg_height_mm sifir/negatifse
                (bunlarla bolme yapilir -- sifirsa sonuc matematiksel olarak
                tanimsizdir, sessizce inf/NaN uretmek yerine acikca durdurulur).
        """
        if not np.isfinite(cg_height_mm) or abs(cg_height_mm) < 1e-6:
            raise GeometryError(
                "Agirlik merkezi yuksekligi (cg_height_mm) sifir veya "
                "gecersiz -- anti-dive/squat yuzdesi tanimsiz (sifira bolme)."
            )
        if not np.isfinite(wheelbase_mm):
            raise GeometryError("Dingil mesafesi (wheelbase_mm) gecersiz (NaN/Inf).")
        (pA, dA), (pB, dB) = self.side_view_lines()
        ic = _line_intersect_2d(pA, dA, pB, dB)
        cp = self._p("contact_patch")[[0, 2]]
        dirC = ic - cp
        if not np.isfinite(ic).all() or (abs(dirC[0]) < 1e-9 and abs(dirC[1]) < 1e-9):
            angle = 0.0
        else:
            angle = np.degrees(np.arctan2(dirC[1], dirC[0]))
            # normalize into (-90, 90]: tan() has period 180 deg, so a raw
            # atan2 result near +-180 deg represents the same shallow swing
            # angle as one near 0 deg -- wrap it so the displayed number is
            # the intuitive small angle instead of a scary near-180 value.
            if angle > 90:
                angle -= 180
            elif angle <= -90:
                angle += 180
        key = "anti_dive_pct" if self.axle == "front" else "anti_squat_pct"

        # tan(angle) blows up as |angle| -> 90 deg (CP->IC line nearly
        # vertical in side view). This is a real, if rare, geometric
        # configuration -- not bad input -- but reporting a ~1e18 %% number
        # is worse than useless. Flag it as unreliable instead of returning
        # a number that looks like a bug.
        reliable = abs(angle) <= 85.0
        if reliable:
            pct = 100.0 * np.tan(np.radians(angle)) * wheelbase_mm / cg_height_mm
        else:
            pct = float("nan")

        return {
            "side_view_instant_center_xz": ic,
            "side_view_swing_angle_deg": angle,
            "reliable": reliable,
            key: pct,
        }

    # ---- Serilestirme -----------------------------------------------------

    def to_dict(self) -> dict:
        d = asdict(self)
        d["_type"] = type(self).__name__
        return d

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2)

    @staticmethod
    def from_dict(d: dict) -> "Suspension":
        type_name = d.get("_type", "McPherson")
        cls = SUSPENSION_TYPES.get(type_name)
        if cls is None:
            raise ValueError(f"Bilinmeyen suspansiyon tipi: {type_name}")
        valid = {f.name for f in fields(cls)}
        # JSON round-trip turns Vec3 tuples into lists; convert back so
        # equality/type expectations (Vec3 = Tuple[float,float,float]) hold.
        clean = {}
        for k, v in d.items():
            if k not in valid:
                continue
            clean[k] = tuple(v) if isinstance(v, list) else v
        return cls(**clean)

    @staticmethod
    def from_json(s: str) -> "Suspension":
        return Suspension.from_dict(json.loads(s))


# --------------------------------------------------------------------------
# McPherson
# --------------------------------------------------------------------------

@dataclass
class McPherson(Suspension):
    """Varsayilan degerler gercek arac hardpoint verisidir (kullanici
    tarafindan saglandi). Sag taraf (outboard=+Y) kabulune gore saklanir;
    kaynak veri sol taraf icindi (-Y), burada aynalanarak (Y*-1) saklandi --
    side='L' secildiginde _p() tekrar aynalayip orijinal sol-taraf
    koordinatlarina donduruyor."""

    lca_front: Vec3 = (60.0, 400.0, 190.0)          # ic pivot - on (sase)
    lca_rear: Vec3 = (460.0, 390.0, 205.0)          # ic pivot - arka (sase)
    lca_outer: Vec3 = (240.0, 700.0, 175.0)         # dis rot bilyesi P1 (knuckle)
    strut_lwr_mount: Vec3 = (300.0, 600.0, 290.0)   # strut alt montaj S (knuckle)
    top_mount: Vec3 = (317.5, 580.0, 755.0)         # strut ust montaj T (sase)
    tierod_inner: Vec3 = (467.0, 400.0, 330.0)      # kremayer ucu (sase)
    tierod_outer: Vec3 = (410.0, 690.0, 300.0)      # dis rot bilyesi P2 (knuckle)
    wheel_center: Vec3 = (260.0, 776.0, 340.0)
    contact_patch: Vec3 = (260.0, 776.0, 0.0)

    KINGPIN_AXIS_EXACT = True

    def kingpin_axis(self):
        return self._p("top_mount"), self._p("lca_outer")

    def front_view_lines(self):
        A = self._p("lca_front")[[1, 2]]
        B = self._p("lca_rear")[[1, 2]]
        pivot = 0.5 * (A + B)
        P1 = self._p("lca_outer")[[1, 2]]
        T = self._p("top_mount")[[1, 2]]
        dA = P1 - pivot
        strut_dir = P1 - T
        dB = _perp_2d(strut_dir)
        return (pivot, dA), (T, dB)

    def side_view_lines(self):
        A = self._p("lca_front")[[0, 2]]
        B = self._p("lca_rear")[[0, 2]]
        pivot = 0.5 * (A + B)
        P1 = self._p("lca_outer")[[0, 2]]
        T = self._p("top_mount")[[0, 2]]
        dA = P1 - pivot
        strut_dir = P1 - T
        dB = _perp_2d(strut_dir)
        return (pivot, dA), (T, dB)

    def plot_points(self):
        pts = super().plot_points()
        pts.update({
            "LCA front bushing": self._p("lca_front"),
            "LCA rear bushing": self._p("lca_rear"),
            "LCA outer (P1)": self._p("lca_outer"),
            "Top mount (T)": self._p("top_mount"),
            "Strut lower mount (S)": self._p("strut_lwr_mount"),
        })
        return pts

    def plot_edges(self):
        edges = super().plot_edges()
        edges += [
            (self._p("lca_front"), self._p("lca_outer"), "tab:blue", "-", "LCA"),
            (self._p("lca_rear"), self._p("lca_outer"), "tab:blue", "-", None),
            (self._p("top_mount"), self._p("strut_lwr_mount"), "tab:orange", "-", "Strut"),
        ]
        return edges

    def geometry_notes(self):
        return super().geometry_notes() + [
            "Kingpin ekseni tamdir: top_mount (T) ile lca_outer (P1) fiziksel "
            "olarak direksiyon donusunun gerceklestigi iki noktadir."
        ]


# --------------------------------------------------------------------------
# Double Wishbone
# --------------------------------------------------------------------------

@dataclass
class DoubleWishbone(Suspension):
    """Varsayilan degerler gercek arac hardpoint verisidir (kullanici
    tarafindan saglandi); McPherson'daki gibi sag-taraf (outboard=+Y)
    kabulune gore aynalanarak saklandi."""

    uca_front: Vec3 = (167.0, 350.0, 585.0)   # UCA ic pivot - on (sase)
    uca_rear: Vec3 = (517.0, 390.0, 560.0)    # UCA ic pivot - arka (sase)
    uca_outer: Vec3 = (307.0, 575.0, 555.0)   # UCA dis top mafsal (knuckle)
    lca_front: Vec3 = (67.0, 350.0, 210.0)    # LCA ic pivot - on (sase)
    lca_rear: Vec3 = (467.0, 350.0, 215.0)    # LCA ic pivot - arka (sase)
    lca_outer: Vec3 = (267.0, 640.0, 155.0)   # LCA dis top mafsal (knuckle)
    tierod_inner: Vec3 = (467.0, 360.0, 330.0)
    tierod_outer: Vec3 = (417.0, 620.0, 330.0)
    wheel_center: Vec3 = (267.0, 760.0, 330.0)
    contact_patch: Vec3 = (267.0, 760.0, 0.0)

    KINGPIN_AXIS_EXACT = True

    def kingpin_axis(self):
        return self._p("uca_outer"), self._p("lca_outer")

    def front_view_lines(self):
        Au, Bu = self._p("uca_front")[[1, 2]], self._p("uca_rear")[[1, 2]]
        pivotU = 0.5 * (Au + Bu)
        Ou = self._p("uca_outer")[[1, 2]]
        Al, Bl = self._p("lca_front")[[1, 2]], self._p("lca_rear")[[1, 2]]
        pivotL = 0.5 * (Al + Bl)
        Ol = self._p("lca_outer")[[1, 2]]
        return (pivotU, Ou - pivotU), (pivotL, Ol - pivotL)

    def side_view_lines(self):
        Au, Bu = self._p("uca_front")[[0, 2]], self._p("uca_rear")[[0, 2]]
        pivotU = 0.5 * (Au + Bu)
        Ou = self._p("uca_outer")[[0, 2]]
        Al, Bl = self._p("lca_front")[[0, 2]], self._p("lca_rear")[[0, 2]]
        pivotL = 0.5 * (Al + Bl)
        Ol = self._p("lca_outer")[[0, 2]]
        return (pivotU, Ou - pivotU), (pivotL, Ol - pivotL)

    def plot_points(self):
        pts = super().plot_points()
        pts.update({
            "UCA front bushing": self._p("uca_front"),
            "UCA rear bushing": self._p("uca_rear"),
            "UCA outer": self._p("uca_outer"),
            "LCA front bushing": self._p("lca_front"),
            "LCA rear bushing": self._p("lca_rear"),
            "LCA outer": self._p("lca_outer"),
        })
        return pts

    def plot_edges(self):
        edges = super().plot_edges()
        edges += [
            (self._p("uca_front"), self._p("uca_outer"), "tab:orange", "-", "UCA"),
            (self._p("uca_rear"), self._p("uca_outer"), "tab:orange", "-", None),
            (self._p("lca_front"), self._p("lca_outer"), "tab:blue", "-", "LCA"),
            (self._p("lca_rear"), self._p("lca_outer"), "tab:blue", "-", None),
        ]
        return edges

    def geometry_notes(self):
        return super().geometry_notes() + [
            "Kingpin ekseni tamdir: uca_outer ve lca_outer, iki ust/alt top "
            "mafsalidir; roll-center ve anti-geometry insaasi UCA+LCA "
            "hatlarinin (klasik cift-salincak yontemi) kesisimini kullanir."
        ]


# --------------------------------------------------------------------------
# Multi-link (orn. 5-link)
# --------------------------------------------------------------------------

@dataclass
class MultiLink(Suspension):
    upper_control_link_chassis: Vec3 = (50.0, 250.0, 480.0)
    upper_control_link_knuckle: Vec3 = (20.0, 700.0, 460.0)
    lower_control_link_chassis: Vec3 = (-50.0, 240.0, 140.0)
    lower_control_link_knuckle: Vec3 = (10.0, 715.0, 130.0)
    toe_link_chassis: Vec3 = (100.0, 300.0, 180.0)
    toe_link_knuckle: Vec3 = (110.0, 700.0, 170.0)
    trailing_link_chassis: Vec3 = (-350.0, 260.0, 200.0)
    trailing_link_knuckle: Vec3 = (50.0, 690.0, 220.0)
    camber_link_chassis: Vec3 = (30.0, 260.0, 350.0)
    camber_link_knuckle: Vec3 = (15.0, 705.0, 340.0)

    axle: str = "rear"
    KINGPIN_AXIS_EXACT = False

    def kingpin_axis(self):
        return self._p("upper_control_link_knuckle"), self._p("lower_control_link_knuckle")

    def front_view_lines(self):
        Cu = self._p("upper_control_link_chassis")[[1, 2]]
        Ku = self._p("upper_control_link_knuckle")[[1, 2]]
        Cl = self._p("lower_control_link_chassis")[[1, 2]]
        Kl = self._p("lower_control_link_knuckle")[[1, 2]]
        return (Cu, Ku - Cu), (Cl, Kl - Cl)

    def side_view_lines(self):
        # Yaklasik: trailing_link (boylamsal kuvveti tasiyan ana link) +
        # lower_control_link. toe_link/camber_link/upper_control_link'in tam
        # 3D katkisi ihmal edilir -- gercek instantaneous eksen icin tam bir
        # 5-link multibody cozumu gerekir (bkz. geometry_notes).
        Ct = self._p("trailing_link_chassis")[[0, 2]]
        Kt = self._p("trailing_link_knuckle")[[0, 2]]
        Cl = self._p("lower_control_link_chassis")[[0, 2]]
        Kl = self._p("lower_control_link_knuckle")[[0, 2]]
        return (Ct, Kt - Ct), (Cl, Kl - Cl)

    def plot_points(self):
        pts = super().plot_points()
        pts.update({
            "Upper control link - chassis": self._p("upper_control_link_chassis"),
            "Upper control link - knuckle": self._p("upper_control_link_knuckle"),
            "Lower control link - chassis": self._p("lower_control_link_chassis"),
            "Lower control link - knuckle": self._p("lower_control_link_knuckle"),
            "Toe link - chassis": self._p("toe_link_chassis"),
            "Toe link - knuckle": self._p("toe_link_knuckle"),
            "Trailing link - chassis": self._p("trailing_link_chassis"),
            "Trailing link - knuckle": self._p("trailing_link_knuckle"),
            "Camber link - chassis": self._p("camber_link_chassis"),
            "Camber link - knuckle": self._p("camber_link_knuckle"),
        })
        return pts

    def plot_edges(self):
        edges = super().plot_edges()
        edges += [
            (self._p("upper_control_link_chassis"), self._p("upper_control_link_knuckle"), "tab:orange", "-", "Upper control link"),
            (self._p("lower_control_link_chassis"), self._p("lower_control_link_knuckle"), "tab:blue", "-", "Lower control link"),
            (self._p("toe_link_chassis"), self._p("toe_link_knuckle"), "tab:green", "-", "Toe link"),
            (self._p("trailing_link_chassis"), self._p("trailing_link_knuckle"), "tab:brown", "-", "Trailing link"),
            (self._p("camber_link_chassis"), self._p("camber_link_knuckle"), "tab:purple", "-", "Camber link"),
        ]
        return edges

    def geometry_notes(self):
        return super().geometry_notes() + [
            "YAKLASIK MODEL: multi-link icin tek bir sabit kingpin ekseni "
            "fiziksel olarak yoktur (5 link birlikte knuckle'in aninlik "
            "donme eksenini belirler ve bu eksen tekerlek hareketiyle "
            "kayar). Burada KPI/caster/scrub/trail, upper_control_link ve "
            "lower_control_link noktalarini bir 'esdeger' kingpin ekseni "
            "gibi kullanarak yaklasik hesaplanir.",
            "Roll-center insaasi ayni sekilde upper_control_link + "
            "lower_control_link hatlarini kullanir (cift-salincak "
            "benzetmesi).",
            "Anti-squat/dive tahmini trailing_link + lower_control_link "
            "hatlarini kullanir; toe_link, camber_link ve "
            "upper_control_link'in katkisi ihmal edilir.",
            "Kesin sonuc icin tam bir 5-link multibody (rijit govde) "
            "cozucusu gerekir -- bu surumde YOK.",
        ]


SUSPENSION_TYPES = {
    "McPherson": McPherson,
    "DoubleWishbone": DoubleWishbone,
    "MultiLink": MultiLink,
}

SUSPENSION_TYPE_LABELS_TR = {
    "McPherson": "McPherson",
    "DoubleWishbone": "Double Wishbone (Cift Salincak)",
    "MultiLink": "Multi-link (Cok Link)",
}


# --------------------------------------------------------------------------
# Arac seviyesi (yarim/tam tasit) agregasyon fonksiyonlari
# --------------------------------------------------------------------------

def track_width_mm(corner: Suspension) -> float:
    """Bu koseden simetrik iz genisligi tahmini (|Y| * 2)."""
    return 2.0 * abs(corner._p("wheel_center")[1])


def wheelbase_mm(front: Suspension, rear: Suspension) -> float:
    return float(front._p("wheel_center")[0] - rear._p("wheel_center")[0])


def roll_axis(front: Suspension, rear: Suspension) -> dict:
    """On ve arka roll-center yukseklikleri + dingil mesafesinden roll ekseni
    (side-view) egimi."""
    rc_f = front.front_view_roll_center()["roll_center_height_mm"]
    rc_r = rear.front_view_roll_center()["roll_center_height_mm"]
    wb = wheelbase_mm(front, rear)
    if abs(wb) < 1e-6:
        angle = float("nan")
    else:
        angle = np.degrees(np.arctan2(rc_f - rc_r, wb))
    return {
        "front_rc_height_mm": rc_f,
        "rear_rc_height_mm": rc_r,
        "wheelbase_mm": wb,
        "roll_axis_angle_deg": angle,
    }


# --------------------------------------------------------------------------
# Sanity check / self-test (`python kinematics.py`)
# --------------------------------------------------------------------------

def _assert_finite(value, msg):
    assert np.isfinite(value), f"BEKLENMEYEN NaN/Inf: {msg} -> {value}"


def _assert_range(value, lo, hi, msg):
    assert lo <= value <= hi, f"ARALIK DISI: {msg} -> {value} (beklenen [{lo},{hi}])"


def _run_sanity_checks():
    print("=== Sanity check baslıyor ===\n")

    for name in ("McPherson", "DoubleWishbone", "MultiLink"):
        cls = SUSPENSION_TYPES[name]
        print(f"--- {name} ---")
        r = cls(side="R", axle="front")
        l = cls(side="L", axle="front")

        ar = r.static_angles()
        al = l.static_angles()

        # 1) NaN/Inf yok
        for label, ang in (("R", ar), ("L", al)):
            for key in ("kpi_deg", "caster_deg", "scrub_radius_mm", "mechanical_trail_mm"):
                _assert_finite(ang[key], f"{name}/{label}/{key}")

        # 2) KPI/caster makul aralikta (formul geregi matematiksel olarak
        #    [0,90] disinda cikamaz; regresyon bekcisi olarak assert ediyoruz)
        for label, ang in (("R", ar), ("L", al)):
            _assert_range(ang["kpi_deg"], 0.0, 90.0, f"{name}/{label}/KPI")
            _assert_range(ang["caster_deg"], 0.0, 90.0, f"{name}/{label}/Caster")

        # 3) Sag/Sol Y isareti dogru mu? (R: +Y disa, L: -Y disa)
        wc_r = r._p("wheel_center")
        wc_l = l._p("wheel_center")
        assert wc_r[1] > 0, f"{name}: R tarafinda wheel_center.Y pozitif olmali, {wc_r[1]} bulundu"
        assert wc_l[1] < 0, f"{name}: L tarafinda wheel_center.Y negatif olmali, {wc_l[1]} bulundu"
        assert abs(wc_r[1]) == abs(wc_l[1]), f"{name}: R/L wheel_center.Y buyuklugu esit olmali"

        # 4) Simetrik bir arac icin scrub radius ve roll-center yuksekligi
        #    R ve L'de AYNI (fiziksel olarak ayni tasarim, sadece aynalanmis)
        assert abs(ar["scrub_radius_mm"] - al["scrub_radius_mm"]) < 1e-6, (
            f"{name}: scrub radius R/L'de farkli cikti ({ar['scrub_radius_mm']} vs "
            f"{al['scrub_radius_mm']}) -- aynalama isareti hatali olabilir"
        )
        rc_r = r.front_view_roll_center()["roll_center_height_mm"]
        rc_l = l.front_view_roll_center()["roll_center_height_mm"]
        assert abs(rc_r - rc_l) < 1e-6, f"{name}: roll-center yuksekligi R/L'de farkli ({rc_r} vs {rc_l})"

        # 5) anti_geometry: 'reliable' bayragiyla tutarli olmali -- guvenilirse
        #    pct sonlu, degilse (|angle| -> 90 deg, tan() patlar) NaN olmali;
        #    hicbir durumda astronomik/anlamsiz bir sayi donmemeli.
        anti = r.anti_geometry(wheelbase_mm=2600, cg_height_mm=520)
        pct_key = "anti_dive_pct" if r.axle == "front" else "anti_squat_pct"
        if anti["reliable"]:
            _assert_finite(anti[pct_key], f"{name}/anti/{pct_key}")
            _assert_range(abs(anti[pct_key]), 0.0, 10000.0, f"{name}/anti/{pct_key} (guvenilir ama asiri buyuk)")
        else:
            assert np.isnan(anti[pct_key]), f"{name}: reliable=False iken {pct_key} NaN olmali, {anti[pct_key]} bulundu"
            print(f"  (not: side-view swing angle {anti['side_view_swing_angle_deg']:.1f} deg, "
                  f"90 deg'ye cok yakin -> anti-% guvenilmez olarak isaretlendi, NaN raporlandi)")

        print(f"  KPI={ar['kpi_deg']:.2f} deg  Caster={ar['caster_deg']:.2f} deg  "
              f"Scrub={ar['scrub_radius_mm']:.1f} mm  Trail={ar['mechanical_trail_mm']:.1f} mm  "
              f"RollCenter={rc_r:.1f} mm")
        print(f"  R/L mirroring OK, KPI/Caster araliginda, NaN/Inf yok.  [PASS]\n")

    # 5) Bozuk/dejenere veri -> GeometryError firlatilmali (sessiz NaN degil)
    print("--- Hata yakalama testleri ---")
    broken = McPherson(top_mount=(240.0, 700.0, 175.0), lca_outer=(240.0, 700.0, 175.0))  # T == P1
    try:
        broken.static_angles()
        raise AssertionError("BEKLENEN HATA FIRLAMADI: cakisik T/P1 icin GeometryError bekleniyordu")
    except GeometryError as e:
        print(f"  Cakisik kingpin ekseni -> GeometryError yakalandi: {e}  [PASS]")

    ok = McPherson()
    try:
        ok.anti_geometry(wheelbase_mm=2600, cg_height_mm=0.0)
        raise AssertionError("BEKLENEN HATA FIRLAMADI: cg_height_mm=0 icin GeometryError bekleniyordu")
    except GeometryError as e:
        print(f"  cg_height_mm=0 -> GeometryError yakalandi: {e}  [PASS]")

    print("\n=== TUM SANITY CHECK'LER GECTI ===")


if __name__ == "__main__":
    _run_sanity_checks()

    print("\n=== Ornek ciktilar (R, front) ===")
    for name, cls in SUSPENSION_TYPES.items():
        print(f"\n--- {name} ---")
        s = cls(side="R", axle="front")
        print("static_angles:", s.static_angles())
        print("roll_center:", s.front_view_roll_center())
        print("anti:", s.anti_geometry(wheelbase_mm=2600, cg_height_mm=520))
        for note in s.geometry_notes():
            print("note:", note)
