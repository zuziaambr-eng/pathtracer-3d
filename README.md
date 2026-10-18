# Path Tracer 3D (C++ / Win32 GUI / CUDA)

Kompletny zestaw narzędzi do renderowania metodą **Path Tracingu** (Monte Carlo) oraz **Dwukierunkowego Path Tracingu (BDPT)** ze skomplikowaną kaustyką optyczną.

## Kluczowe możliwości aplikacji GUI (`pathtracer_gui.exe`)

1. **Pełne wsparcie maksymalizacji i skalowania okna (`WS_OVERLAPPEDWINDOW`)**:
   - Aplikację można teraz **zmaksymalizować na cały ekran** (przycisk maksymalizacji w prawym górnym rogu okna lub dwuklik na belkę).
   - Kanwa widoku 3D automatycznie i dynamicznie rozszerza się, dopasowując rozdzielczość i proporcje kadru w czasie rzeczywistym.
   - Prawy panel sterowania pozostaje wygodnie zadokowany przy prawej krawędzi.

2. **Naprawa orientacji podglądu (Prawidłowy układ Top-Down)**:
   - Wyeliminowano odwrócenie obrazu — góra kadru to zawsze góra ekranu, a ruch kursora myszki odpowiada bezpośrednio kierunkowi przemieszczania w przestrzeni 3D.
   - Wskaźniki zaznaczenia (pomarańczowy pierścień) i linie osi idealnie pokrywają się z bryłami 3D co do piksela.

3. **Nawigacja kamerą 3D w stylu Blendera**:
   - **Obrót kamery (Orbit)**: Przytrzymaj **Środkowy Przycisk Myszy (ŚPM / kółko)** lub **Alt + LPM** i przeciągnij, aby płynnie orbitować kamerą wokół sceny.
   - **Przybliżanie / Oddalanie (Zoom)**: Kręć **kółkiem myszy (Scroll)**, aby zbliżać lub oddalać widok.
   - **Przesuwanie widoku (Pan)**: Przytrzymaj **Shift + ŚPM** i przeciągnij, aby przesuwać punkt skupienia kamery.
   - **Reset kamery**: Przycisk `🎥 Reset kamery 3D` natychmiast przywraca optymalny kąt początkowy.

4. **Transformacje obiektów w stylu Blendera (G, S, osie X/Y/Z)**:
   - **Wybór obiektu**: Kliknij **LPM** na dowolny obiekt w kadrze (Ray Picking).
   - **Klawisz G (Grab / Przesuwanie)**: Przesuwa obiekt w 3D pod kursorem myszy.
   - **Klawisz S (Scale / Skalowanie)**: Skaluje promień/rozmiar zaznaczonego obiektu ruchem myszki.
   - **Blokada osi (X, Y, Z)**: Wciśnięcie **X**, **Y** lub **Z** w trakcie przesuwania blokuje ruch do danej osi z wyświetleniem kolorowej linii prowadnicy 3D (Czerwona X, Zielona Y, Niebieska Z).
   - **Klawisz F (Floor Grid)**: Włącza/wyłącza perspektywiczną siatkę podłoża w stylu Blendera.
   - **Zatwierdzenie / Anulowanie**: **LPM** / **Enter** zatwierdza, **PPM** / **Escape** anuluje transformację.

5. **Nowa geometria sceny: Szlifowany Kryształ Szklany (Faceted Crystal / Polyhedron)**:
   - 20-fasetowy szlifowany kryształ (`ConvexPolyhedron`) oparty na analitycznym obcinaniu półprzestrzeni (Slab Method).
   - Rzuca na podłoże skomplikowaną sieć zbieżnych pasm świetlnych, diamentowych skupień i promieni gwiazdy (*starburst caustics*).

6. **Dramatyczny kąt oświetlenia (Boczno-Tylny)**:
   - Źródło światła umieszczone pod niskim kątem ($X=3.2, Y=2.4, Z=-1.2$).
   - Promienie przechodzą przez fasety kryształu pod skosem, rzucając wydłużony wachlarz kaustyki na podłogę.

7. **Brak szwów kafelkowania (Globalna Mapa Fotonów)**:
   - Ciągła siatka przestrzenna $O(1)$ gwarantuje spójną kaustykę bez jakichkolwiek szwów kafelkowania niezależnie od rozmiaru kafelka (16x16, 32x32, 64x64).

8. **Konfiguracja ilości fotonów i własna nazwa pliku PNG**:
   - Pełna edycja ilości fotonów kaustyki (`IDC_EDIT_PHOTONS`) oraz nazwy pliku przed zapisem (`IDC_EDIT_FILENAME`).

9. **Metropolis Light Transport (MLT) – David Cline & Parris Egbert (2005)**:
   - Dodano zaawansowaną metodę transportu światła **MLT** opartą na łańcuchach Markowa (MCMC) i zasadzie równowagi szczegółowej (*Detailed Balance*).
   - **Nowa ścieżka (Large Step Mutation, Sekcja 3.2)**: Losuje zupełnie nową ścieżkę światła, gwarantując ergodyczność i eksplorację całej sceny.
   - **Perturbacja soczewki (Lens Perturbation, Appendix B)**: Wykładniczo rozproszone przesunięcie punktu na płaszczyźnie projekcji ($r = r_2 \cdot e^{-\ln(r_2/r_1)\xi}$), pozwalające na gęste próbkowanie pasm kaustycznych.
   - **Perturbacja kątowa odbić (Angular Perturbation, Appendix C)**: Wykładnicze zaburzenie kąta odbicia i załamania promieni ($\theta = \theta_2 \cdot e^{-\ln(\theta_2/\theta_1)\xi}$).
   - **Akumulacja wartości oczekiwanej (Expected Value Accumulation, Sekcja 5.1)**: Obie propozycje ($Y$ oraz $X$) wpłacają swój wkład ważony prawdopodobieństwem akceptacji $a(y|x) = \min(1, F_y/F_x)$, drastycznie redukując szum.
   - **Faza Bootstrap & Skalowanie (Sekcja 3.5 & Równanie 1)**: Oszacowanie średniej luminancji $f_{ave}$ i skalowanie histogramu próbek $s = f_{ave} / h_{ave}$.
   - **Równoległość MCMC**: Niezależne łańcuchy Markowa na wszystkich rdzeniach procesora z aktualizacją podglądu w czasie rzeczywistym.

10. **Manifold Exploration (MEMLT / MEPT) – Wenzel Jakob & Steve Marschner (ACM TOG / SIGGRAPH 2012)**:
    - Dodano najnowocześniejszą metodę **Manifold Exploration**, rozwiązującą problem renderowania trudnych ścieżek zwierciadlanych i refrakcyjnych ($S^*$).
    - **Rozmaitość zwierciadlana (*Specular Manifold*, Sekcja 3)**: Zbiór fizycznie dopuszczalnych ścieżek spełniających uogólnione prawo Snella / Fermata tworzy podrozmaitość w przestrzeni ścieżek ($C(\bar{x}) = 0$).
    - **Algorytm `WALKMANIFOLD` (Sekcja 4 & Rysunek 4)**: Dwuetapowy predyktor-korektor (metoda Newtona) poruszający się w przestrzeni stycznej rozmaitości za pomocą Jacobianu różniczkowego ($J$) i rzutujący promienie z powrotem na rozmaitość za pomocą śledzenia promieni ze zbieżnością kwadratową.
    - **Perturbacja rozmaitości (*Manifold Perturbation*, Sekcja 5.1 & Rysunek 5, 6)**: Płynne poruszanie się wzdłuż krawędzi i pasm kaustycznych szlifowanego kryształu bez utraty połączenia ze źródłem światła.
    - **Test odwracalności (*Reversibility*, Sekcja 5.1)**: Weryfikacja powrotnego spaceru gwarantuje pełną odwracalność i zachowanie Detailed Balance w łańcuchu Markowa.

11. **Druga Scena Testowa: Zamknięty Pokój ze Szczeliną Okienną (Benchmark Manifold Exploration)**:
    - Kamera umieszczona wewnątrz zamkniętego pokoju.
    - W przedniej ścianie znajduje się wąskie, podłużne okno szczelinowe.
    - Kuliste źródło światła umieszczone na zewnątrz za nieprzezroczystą przesłoną (*baffle*) – brak bezpośredniego widoku ze szczeliny na źródło światła.
    - Światło wpada do wnętrza pokoju **wyłącznie po odbiciu od zewnętrznej ściany zwierciadlanej (lustra)**.
    - Wewnątrz pokoju umieszczony jest szklany kryształ i złota kula, tworzące załamania i kaustykę z wpadającego promienia światła.
    - Klasyczny problem wąskiej apertury i pośredniego transportu zwierciadlanego – idealny do demonstracji potęgi Manifold Exploration (MEMLT) nad klasycznym PT i BDPT!

12. **Interaktywny Edytor Materiałów w Czasie Rzeczywistym**:
    - Możliwość zmiany typu materiału dla dowolnego zaznaczonego obiektu:
      - `Lambertian` (matowy, rozpraszający)
      - `Metal` (zwierciadlany, lustrzany)
      - `Dielectric` (szkło, kryształ, refrakcyjny)
      - `DiffuseLight` (emisyjny, źródło światła)
    - Wygodna edycja parametrów w GUI: kolor RGB, chropowatość (*Roughness* dla metalu), współczynnik załamania (*IOR* dla dielektryka) oraz moc świecenia (*Intensity* dla światła).
    - Przycisk `🎨 Zastosuj Materiał do Wybranego Obiektu` natychmiastowo aktualizuje podgląd 3D.

13. **Uniwersalny Importer Geometrii 3D (Wavefront .OBJ)**:
    - Przycisk `📂 Importuj model 3D Wavefront (.OBJ)` z natywnym oknem dialogowym Windows.
    - Automatyczne parsowanie wierzchołków (`v`) i ścian (`f`), triangulacja polygonów.
    - Automatyczne centrowanie i normalizacja skali siatki do przestrzeni roboczej sceny.
    - Akceleracja AABB (Bounding Box) + szybki algorytm Möllera-Trumbore dla przecięcia promienia z trójkątami.
    - Pełna integracja z silnikiem manipulacji: zaimportowany model można wybierać myszką, przesuwać klawiszem `G`, skalować `S`, blokować osie `X/Y/Z` oraz nakładać na niego dowolne materiały (szkło, złoto, emiter światła, matowy). Dołączono przykładowe pliki `diamond.obj` oraz `test_pyramid.obj`.

---

## Uruchomienie

Dwukrotny klik na **`pathtracer_gui.exe`** w folderze roboczym lub w terminalu:
```powershell
.\pathtracer_gui.exe
```

### Kompilacja (LLVM Clang++):
```powershell
clang++ -std=c++17 -O3 gui_main.cpp -o pathtracer_gui.exe -lgdi32 -luser32 -lcomctl32 -lcomdlg32 -mwindows
```
