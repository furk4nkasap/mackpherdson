# McPherson Suspansiyon Hardpoint Analiz Araci (Streamlit)

Hardpoint koordinatlarini girdiginde front-view roll-center insaasini, kingpin
ekseninden turetilen KPI / caster / scrub radius / mekanik trail degerlerini,
ve bir bump/rebound taramasi ile camber / toe / roll-center-yuksekligi
egrilerini hesaplayip cizen bir Streamlit uygulamasi. Adams/Car Table
Builder'da hardpoint iterasyonundan once (veya paralelinde) hizli bir
"noktalari gir, geometriyi gor" araci olarak dusunulmustur.

## Kurulum ve calistirma

```bash
pip install -r requirements.txt
streamlit run app.py
```

## Dosyalar

- `kinematics.py` — tum geometri/kinematik matematik burada. Bagimsiz da
  calistirilabilir (`python kinematics.py`) ve ornek hardpoint seti icin
  statik acilari + bir sweep sonucunu terminale yazdirir. Modulun basindaki
  docstring (uygulama icinde "Metodoloji & JSON" sekmesinde de gosterilir)
  kullanilan rijit-govde modelini ve tum varsayimlari detayli anlatir.
- `plotting.py` — matplotlib ile 3D linkage gorunumu, front-view roll-center
  insaasi, camber/toe egrileri ve roll-center-vs-travel grafikleri.
- `app.py` — Streamlit arayuzu (sidebar'dan hardpoint girisi, JSON
  import/export, sonuc metrikleri, sekmeli grafikler).

## Yontem ozeti

Knuckle (poyra) rijit bir govde olarak modellenir; uzerinde LCA dis rot
bilyesi (P1), rot kolu dis bilyesi (P2) ve strut-knuckle kelepce referans
noktasi (S) tasir. LCA, P1'i on/arka bushing eksenli bir cember uzerine
kisitlar (bu cemberdeki aci = taramanin surucu degiskeni). Strut, T (ust
montaj, sase sabit) noktasinda kureesel bir mafsal + teleskopik kayma olarak
davranir — bu da knuckle'a sabit "S'den gecen ve knuckle govdesine gore sabit
yonlu bir dogru, her zaman T'den gecmeli" kisitini getirir (2 denklem). Rot
kolu, sabit R (kremayer) noktasina sabit uzunlukta baglanir (1 denklem). Bu 3
denklem + P1'in cember uzerindeki konumu, her tarama adiminda knuckle'in tam
rotasyonunu (3 bilinmeyen) belirler; `scipy.optimize.least_squares`
(Levenberg-Marquardt, sikistirilmis tolerans, bir onceki adimdan warm-start)
ile sayisal olarak cozulur.

KPI, caster, scrub radius ve mekanik trail dogrudan kingpin ekseninden (T→P1
dogrusu) turetilir — bu eksen etrafinda rotasyon fiziksel olarak direksiyon
donusunun ta kendisidir (bkz. `kinematics.py` docstring'indeki DOF analizi).
Front-view roll-center, klasik McPherson el-cizimi yontemiyle bulunur: LCA
hattinin (Line A) ve T noktasindan strut eksenine dik cizilen hattin (Line
B) kesisimi Instant Center'i (IC) verir; lastik temas noktasindan IC'ye
cizilen hat arac merkez hattini roll-center yukseklikte keser.

## Onemli varsayimlar / sinirlamalar

- Tum baglantilar rijit kabul edilir (bushing compliance yok).
- Bump/rebound taramasi sirasinda direksiyon sabit tutulur (kremaye/R
  hareket etmez); direksiyon sweep'i bu surumde yok.
- LCA, iki ic bushing'inin olusturdugu tek bir eksen etrafinda donen rijit
  bir kol olarak idealize edilir.
- Camber/toe, knuckle rotasyonunun sase-sabit X/Z eksenlerine gore (extrinsic
  XYZ Euler) ayristirilmasindan raporlanir — kucuk-orta acilarda (tipik
  +/-80-100mm bump/rebound) pratikte yeterince dogru bir yaklasimdir, ama tam
  bir "wheel-plane normal" hesaplamasi degildir.
- Anti-dive tahmini, tam bir side-view instant-center linkage cozumu yerine
  basitlestirilmis bir LCA salinim-acisi yontemi kullanir — gosterge
  niteliginde, kesin degil.
- Varsayilan (uygulama ilk acildiginda gelen) hardpoint'ler **gercek bir
  araca ait degildir**; sadece makul araliklarda (KPI~11°, caster~5° vb.)
  sonuc uretsin diye elle secilmis ornek degerlerdir. Kendi Adams/CAD
  hardpoint'lerinizle degistirin.
- Ornek varsayilan rot kolu noktalari **optimize edilmemistir** — toe
  egrisinin (bump steer) olmasi gerekenden dik cikmasi bir hata degil, tam da
  bu aracin gostermesi gereken sey: rot kolu ucunu (tierod_outer) Y/Z'de
  oynatarak egriyi duzlestirmeyi deneyin.

## Genisletme fikirleri

- Sag/sol simetri (Y ekseninde ayna) ekleyip her iki tekerlegi ayni anda
  cizmek.
- Direksiyon sweep'i (R noktasini hareket ettirip Ackermann/bump-steer'i
  birlikte incelemek).
- ARB (stabilizatör) baglanti noktalarini ekleyip roll-stiffness dagilimini
  hesaplamak.
- Motion ratio (yay/damper) egrisini strut sikismasindan turetmek.
- Hedef bir camber/toe egrisine gore hardpoint'i otomatik optimize eden bir
  "optimizasyon modu" (scipy.optimize ile disaridan sarmalama).
