# Path Tracer 3D (C++ / Win32 GUI / CUDA)

Kompletny zestaw narzędzi do renderowania metodą **Path Tracingu** (Monte Carlo) oraz **Dwukierunkowego Path Tracingu (BDPT)** ze skomplikowaną kaustyką optyczną.

## Kluczowe możliwości aplikacji GUI (`pathtracer_gui.exe`)

1. **Nowa geometria sceny: Szlifowany Kryształ Szklany (Faceted Crystal / Polyhedron)**:
   - Zastąpiono kule i pierścienie **20-fasetowym szlifowanym kryształem szklanym** (`ConvexPolyhedron`).
   - Geometria oparta na analitycznym obcinaniu półprzestrzeni (Slab Method):
     1. Podstawa i szczyt,
     2. 6 pionowych faset pryzmatu heksagonalnego,
     3. 6 ukośnych faset wierzchołkowych (szlif ostrosłupowy / diamentowy),
     4. 6 dolnych faset ściętych (fazowanie krawędzi podstawy).
   - **Efekt kaustyczny**: Światło załamuje się wielokierunkowo na ostrych krawędziach, rzucając na podłoże skomplikowaną sieć zbieżnych pasm świetlnych, diamentowych skupień i promieni gwiazdy (*starburst caustics*).

2. **Dramatyczny kąt oświetlenia (Boczno-Tylny)**:
   - Źródło światła umieszczono pod niskim kątem z prawej strony i z tyłu kryształu ($X=3.2, Y=2.4, Z=-1.2$).
   - Promienie przechodzą przez fasety kryształu pod skosem, rzucając wydłużony, niezwykle efektowny wachlarz kaustyki na podłogę bezpośrednio w pole widzenia kamery.

3. **Naprawa niespójności kafelkowania (Eliminacja „kratkowanego obrazu”)**:
   - Wprowadzono **Globalną Mapę Fotonów (Pre-Pass)** z ciągłą siatką przestrzenną $O(1)$.
   - W fazie wstępnej fotony są śledzone wielowątkowo z całego źródła światła przez kryształ na scenę.
   - Wszystkie kafelki (zarówno 16x16, 32x32, jak i 64x64) odczytują to samo, globalne pole gęstości strumienia kaustycznego.
   - **Rezultat**: Granice kafelków są w 100% niewidoczne, a obraz jest całkowicie spójny i gładki.

4. **Wybór ilości fotonów kaustyki**:
   - Pole tekstowe **"Fotony kaustyki"** (`IDC_EDIT_PHOTONS`) pozwala ustawić dowolną liczbę fotonów (np. `20000`, `50000`, `150000`).

5. **Własna nazwa pliku przed zapisem**:
   - Pole tekstowe **"Nazwa pliku"** (`IDC_EDIT_FILENAME`) umożliwia wpisanie dowolnej nazwy (domyślnie `render_kaustyka.png`).
   - Aplikacja automatycznie dba o rozszerzenie `.png` oraz informuje o zapisie w oknie komunikatu.

6. **Stochastyczne kafelkowanie (Stochastic Tiling)**:
   - Losowa permutacja pikseli wewnątrz każdego kafelka (`pass-by-pass`).
   - Kolejność renderowania kafelków od środka na zewnątrz (*Center-Out*).

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
