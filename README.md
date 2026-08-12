# Tasit Statik Geometri Hesaplayici (Streamlit)

McPherson, Double Wishbone ve Multi-link suspansiyon tiplerini destekleyen,
nesne yonelimli (OOP) mimariyle yazilmis statik geometri hesaplayicisi.
Ceyrek Tasit / Yarim Tasit / Tam Tasit olmak uzere uc sekmede hardpoint
koordinatlarindan KPI, caster, scrub radius, mekanik trail, roll center,
iz genisligi, dingil mesafesi, roll axis acisi ve anti-dive/squat hesaplar.

Bu surum SADECE STATIK geometriye odaklanir -- onceki tek-tip McPherson
surumundeki bump/rebound sweep/dinamik kinematik cozucu bilerek kaldirildi.
OOP temeli kuruldugu icin ileride bir `DynamicAnalysis` katmani olarak geri
eklenmesi kolaydir (bkz. "Genisletme fikirleri").

## Kurulum ve calistirma

```bash
pip install -r requirements.txt
streamlit run app.py
```

## Dosyalar

- `kinematics.py` — tum siniflar ve saf geometri matematigi (Streamlit'e
  bagimli degil, bagimsiz da calisir: `python kinematics.py`).
- `plotting.py` — matplotlib gorsellestirmeleri; sinif hiyerarsisi
  uzerinden JENERIK calisir, tipe ozel cizim kodu icermez.
- `app.py` — Streamlit arayuzu: 3 ana sekme, tipe gore dinamik hardpoint
  formu, sag/sol aynalama.

## Mimari (OOP)

```
Suspension (taban sinif)
 ├─ ortak alanlar: wheel_center, contact_patch, tierod_inner, tierod_outer,
 │  wheel_radius, static_camber_deg, static_toe_deg, side, axle
 ├─ ortak hesaplar: static_angles(), front_view_roll_center(), anti_geometry()
 ├─ alt siniflarin doldurdugu kancalar: kingpin_axis(), front_view_lines(),
 │  side_view_lines(), plot_points(), plot_edges(), geometry_notes()
 │
 ├─ McPherson       (lca_front, lca_rear, lca_outer, top_mount, strut_lwr_mount)
 ├─ DoubleWishbone  (uca_front, uca_rear, uca_outer, lca_front, lca_rear, lca_outer)
 └─ MultiLink       (upper/lower_control_link, toe_link, trailing_link,
                      camber_link -- her biri _chassis/_knuckle cifti)
```

Taban sinif KPI/caster/scrub/trail'i **tek yerde** (`static_angles`),
front-view roll-center insaasini **tek yerde** (`front_view_roll_center`) ve
anti-dive/squat'i **tek yerde** (`anti_geometry`) hesaplar; her alt sinif
sadece kendine ozgu iki bilgiyi saglar: kingpin ekseni (iki nokta) ve
front/side-view IC insaasi icin gereken iki dogru. Yeni bir suspansiyon tipi
eklemek icin sadece bu kancalari dolduran yeni bir alt sinif yazmak yeterli.

## Hardpoint isimlendirme (tipe gore)

**Genel (tum tipler):** `wheel_center`, `contact_patch`, `tierod_inner`,
`tierod_outer`, `wheel_radius`.

**McPherson:** `lca_front`, `lca_rear` (sase), `lca_outer` (knuckle, P1),
`top_mount` (sase, T), `strut_lwr_mount` (knuckle referans S, gorsel amacli).

**Double Wishbone:** `uca_front`, `uca_rear` (sase), `uca_outer` (knuckle),
`lca_front`, `lca_rear` (sase), `lca_outer` (knuckle).

**Multi-link:** `upper_control_link_chassis/knuckle`,
`lower_control_link_chassis/knuckle`, `toe_link_chassis/knuckle`,
`trailing_link_chassis/knuckle`, `camber_link_chassis/knuckle`.

## Yontem ozeti (tipe gore)

**Kingpin ekseni:** McPherson icin `top_mount → lca_outer` (bu, direksiyon
donusunun fiziksel olarak gerceklestigi iki noktadir -- top mount rulmani ve
LCA top mafsali; `strut_lwr_mount` sadece 3D gorsellestirmede strut govdesini
cizmek icin kullanilir, kingpin eksenine dahil degildir). Double Wishbone
icin `uca_outer → lca_outer` (iki top mafsal). Bu iki tip icin eksen
**tamdir**. Multi-link icin sabit bir kingpin ekseni fiziksel olarak yoktur
(5 link birlikte aninlik donme eksenini belirler ve bu eksen hareketle
kayar); burada `upper_control_link_knuckle → lower_control_link_knuckle`
**yaklasik esdeger eksen** olarak kullanilir (`KINGPIN_AXIS_EXACT = False`
ile isaretlenir, arayuzde uyari gosterilir).

**Front-view roll-center insaasi:** Line A + Line B kesisimi (Instant
Center) → lastik temas noktasindan IC'ye cizilen hattin arac merkez hattini
kestigi yukseklik. McPherson'da Line B = strut top mount T'den strut
eksenine (top_mount → lca_outer) dik cizilen hat (strut bir sürgü/slider
oldugu icin); Double Wishbone'da Line A/B dogrudan UCA/LCA hatlari (klasik
cift-salincak yontemi); Multi-link'te yaklasik olarak upper/lower control
link hatlari.

**Anti-dive/squat:** Ayni IC yontemi side-view'da (X-Z) tekrarlanir.
McPherson: LCA hatti + strut'a dik hat. Double Wishbone: UCA + LCA hatlari.
Multi-link: trailing_link + lower_control_link hatlari (toe_link,
camber_link ve upper_control_link'in katkisi ihmal edilir -- kesin sonuc
icin tam bir 5-link multibody cozucusu gerekir).

**Camber / Toe:** Kontrol kolu/link hardpoint'lerinden turetilmez --
knuckle uzerindeki tekerlek montaj yuzeyinin acisi, top mafsal
konumlarindan bagimsiz bir tasarim/imalat degeridir. Bu yuzden
`static_camber_deg` / `static_toe_deg` dogrudan girilen (olculen/tasarlanan)
degerlerdir, ama yine de geometrinin bir parcasi olarak raporlanir.

**Sag/Sol aynalama:** Hardpoint'ler her zaman "sag taraf" (outboard = +Y)
kabulune gore girilir. `side='L'` secildiginde Y isaretleri (ve scrub
radius'un ic/dis yorumu) hesap sirasinda otomatik aynalanir; saklanan ham
degerler degismez.

## Varsayilan hardpoint koordinatlari

McPherson ve Double Wishbone siniflarinin varsayilan degerleri **gercek bir
araca ait olcuk hardpoint koordinatlaridir** (mm), sag taraf (outboard=+Y)
kabulune gore saklanir. Multi-link'in varsayilanlari ise (kullanicidan gercek
veri gelmedigi icin) hala **ornek/illustratif** degerlerdir -- bkz. asagida
"Onemli varsayimlar".

`side='L'` secildiginde `_p()` yardimci metodu Y isaretini otomatik aynalar;
saklanan ham (dataclass alanindaki) degerler hep sag-taraf kabulune gore
kalir, degismez.

## Hata yakalama (GeometryError) ve guvenilirlik bayragi

Dejenere/gecersiz hardpoint girisi (orn. iki noktanin cakismasi, kingpin
ekseninin sifir uzunlukta cikmasi, agirlik merkezi yuksekliginin 0
girilmesi) `kinematics.py` icinde `GeometryError` (bir `ValueError` alt
sinifi) firlatir; bu, sessizce NaN/Inf uretmek yerine acik bir hata
mesajidir. `app.py`, riskli her hesap cagrisini `safe_call()` sarmalayicisi
ile cevirir: bir `GeometryError` (veya baska beklenmeyen bir hata)
olustugunda sayfa comek yerine `st.error()` ile aciklayici bir mesaj
gosterir ve o bolumu atlar, geri kalan arayuz calismaya devam eder.

Ayrica `anti_geometry()` (anti-dive/squat) hesabinin side-view swing acisi
±90°'ye cok yaklastiginda (`tan()` matematiksel olarak sonsuza gitmeye
basladiginda) sonuc astronomik/anlamsiz bir sayi yerine `reliable: False`
bayragiyla isaretlenir ve yuzde `NaN` olarak raporlanir; arayuzde bu durum
"N/A (guvenilmez: swing acisi ±90°'ye cok yakin)" olarak gosterilir.

`kinematics.py` dogrudan calistirildiginda (`python kinematics.py`) once
`_run_sanity_checks()` calisir: her tip icin NaN/Inf yoklugu, KPI/Caster'in
[0°,90°] araliginda olmasi, sag/sol Y isaretinin dogru aynalanmasi, simetrik
bir aracta scrub radius/roll-center yuksekliginin R ve L'de ayni cikmasi ve
`reliable` bayraginin tutarliligini dogrular; ayrica cakisik hardpoint ve
`cg_height_mm=0` gibi bozuk girdilerin gercekten `GeometryError` firlattigini
da test eder.

## Sekmeler

1. **Ceyrek Tasit** — Tip + Konum (On/Arka) + Taraf (Sag/Sol) sec, hardpoint
   gir; KPI, caster, camber, toe, scrub radius, mekanik trail + 3D gorunum +
   roll-center insaa diyagrami. JSON olarak indirilebilir/yuklenebilir.
2. **Yarim Tasit** — On ve arka suspansiyonu ayri ayri tanimla (klasik
   "bicycle model"); iz genisligi (on/arka) ve on/arka roll-center
   yukseklikleri.
3. **Tam Tasit** — 4 kose (On Sag/Sol, Arka Sag/Sol); simetrik araclar icin
   "sol tarafi sagdan aynala" secenegi (varsayilan acik) ile tekrar veri
   girisini onler. Dingil mesafesi, roll axis acisi, anti-dive (on) /
   anti-squat (arka) yuzdeleri + 4 kosenin ozet tablosu.

## Onemli varsayimlar / sinirlamalar

- **Sadece statik geometri.** Bump/rebound tarama, motion ratio, bump
  steer/camber-gain egrileri bu surumde yok (bilerek kaldirildi).
- McPherson ve Double Wishbone'un varsayilan hardpoint'leri **gercek bir
  araca ait olculmus koordinatlardir**. Multi-link'in varsayilanlari ise
  hala **ornek/illustratif** degerlerdir (gercek veri saglanmadi) -- her
  durumda anti-dive/squat gibi ciktilar front/side-view IC yonteminin dogasi
  geregi hardpoint'lere cok duyarlidir; kendi Adams/CAD hardpoint'lerinizle
  degistirmeniz onerilir.
- Multi-link'in kingpin ekseni, roll-center'i ve anti-geometry'si **YAKLASIK
  MODELDIR** (bkz. yukarida); arayuzde bu acikca belirtilir.
- Anti-dive/squat hesaplari icin dingil mesafesi ve agirlik merkezi
  yuksekligi kullanici girdisi olarak alinir (hardpoint'lerden turetilemez).
- Tam Tasit'ta L/R aynalama, sadece hardpoint konumlarini aynalar; sol ve
  sag tarafin FARKLI geometriye sahip olmasi gereken (orn. hasarli/ozel
  arac) durumlar icin aynalamayi kapatip 4 koseyi bagimsiz girebilirsiniz.

## Genisletme fikirleri

- Dinamik bump/rebound sweep'i (onceki surumdeki `scipy.optimize`
  tabanli rijit-govde cozucusu) `Suspension` alt siniflarina `sweep()`
  metodu olarak geri eklemek -- kancalar (`kingpin_axis`, link hardpoint'leri)
  zaten hazir oldugu icin her tip icin ayri cozucu yazmaya gerek kalmaz.
- ARB (stabilizatör) hardpoint'leri ve roll-stiffness dagilimi.
- Ackermann yuzdesi (iki on tekerlegin steer geometrisi karsilastirmasi).
- Motion ratio (yay/damper) hesaplamasi.
- Tam Tasit'ta on/arka roll-stiffness dagilimina gore toplam roll gradient
  tahmini (yay/ARB sertligi ek girdi olarak gerekir).
