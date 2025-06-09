 🚗 Autonomiczny Pojazd z STM32

Projekt semestralny z przedmiotu **Systemy Wbudowane i Mikrokontrolery**  
Autor: _[Jakub Podgórski, Michał Dobosz]_  
Numer indeksu: _[21294, 21230]_  
Data rozpoczęcia: _[6.05.2025]_  
Repozytorium zawiera kod, dokumentację oraz materiały projektowe.

---

## 📌 Opis projektu

Celem projektu jest opracowanie modelu autonomicznego pojazdu sterowanego za pomocą mikrokontrolera STM32. Pojazd porusza się w trybie półautomatycznym lub automatycznym, omija przeszkody i/lub śledzi linię. Komunikacja z użytkownikiem odbywa się przez UART (Bluetooth lub przewodowo).

---

## 🛠️ Zastosowane technologie i narzędzia

- **Mikrokontroler:** STM32F103C8T6
- **IDE:** STM32CubeIDE
- **Programowanie:** C (HAL / LL)
- **Sensory:**
  - HC-SR04 (ultradźwiękowy)
  - Sensory optyczne (IR)
- **Zasilanie:** 2x 2s Akumulator Li-Ion 18650
- **Sterownik silników:** L293
- **Komunikacja:** Moduł radiowy 4-kanały 315MHz YK04

---

## ⚙️ Funkcjonalności

- ✅ Napęd sterowany przez PWM z użyciem Timerów
- ✅ Obsługa sensorów ultradźwiękowych (pomiar odległości)
- ✅ Odczyt wartości z sensorów IR (linia / przeszkody) przy użyciu ADC
- ✅ Detekcja kolizji i unikanie przeszkód
- ✅ Zasilanie bateryjne – pełna autonomia
- ✅ Regularne wersjonowanie kodu (min. 1 commit/tydzień)

---

## 📁 Struktura repozytorium


---

## 🔌 Komendy UART

| Komenda | Opis                    |
|--------:|-------------------------|
| `START` | Uruchamia pojazd       |
| `STOP`  | Zatrzymuje pojazd      |
| `LEFT`  | Skręt w lewo           |
| `RIGHT` | Skręt w prawo          |

---

## 🧪 Scenariusze testowe

- [x] Detekcja przeszkody z przodu (sensor HC-SR04)
- [x] Reakcja na białą/czarną linię (IR)
- [x] Komunikacja przez moduł radiowy YK04
- [x] Test zasilania bateryjnego
- [x] Sterowanie ruchem w czasie rzeczywistym

---

## 📸 Demo i zdjęcia

- Zdjęcia pojazdu: [`/Media/photos/`](./Media/photos/)
- Nagranie testów: [Demo Video](#) *(link do YouTube lub Dysku Google)*

---

## 📄 Dokumentacja

Pełna dokumentacja projektu znajduje się w folderze [`Docs/`](./Docs/), w tym:
- Raport końcowy (PDF)
- Schematy układów
- Lista komponentów

---

## 📅 Harmonogram pracy

- Tydzień 1–2: Koncepcja i lista komponentów  
- Tydzień 3–5: Budowa pojazdu i montaż elektroniki  
- Tydzień 6–9: Programowanie sensorów i napędu  
- Tydzień 10–12: Komunikacja UART + testy  
- Tydzień 13–14: Finalizacja, dokumentacja, raport  

---

## 🧠 Wnioski


---

## 📬 Kontakt

W razie pytań:
- Email: _21294@student.ans-elblag.pl_, 21230@student.ans-elblag.pl
- GitHub: [MasterZer0o](https://github.com/MasterZer0o/)

---

**Licencja:** MIT  
