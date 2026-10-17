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

---

## Uruchomienie

Dwukrotny klik na **`pathtracer_gui.exe`** w folderze roboczym lub w terminalu:
```powershell
.\pathtracer_gui.exe
```

### Kompilacja (LLVM Clang++):
```powershell
clang++ -std=c++17 -O3 gui_main.cpp -o pathtracer_gui.exe -lgdi32 -luser32 -lcomctl32 -mwindows
```
