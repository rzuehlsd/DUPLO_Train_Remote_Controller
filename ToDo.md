## Aktuelle ToDos

### 1. Record /Replay Funktion:
- [x] Die Replay Liste wird nicht nur genau einmal abgefahren sondern in einer Schleife -> Sollverhalten: Liste einmal abfahren und dann den Replay Modus beenden
- [x] Während Replay müssen alle Kommandos außer Beenden des Replaymodus und Stop ignoriert werden
- [x] Ebenso sollten die  über den Color Sensor identifizierten Aktionen im Replay Modus NICHT ausgeführt werden
- [x] Im Record Modus jedoch sollen die über den Color Sensor identifizierten Befehle mit aufgezeichnet werden
- [x] Das Timing der aufgezeichneten Befehle muss überprüft werden und sollte bis auf 500ms mit der Aufgezeichneten Sequenz beim Abspielen übereinstimmen

### 2. Sonstige Softwareänderungen
- [x] Der Voltage Sensor muss aktiviert werden um die Spannung der Duplobatterie zu überwachen. Klären bei welchem Soannungslevel eine Warnung ausgegeben werden soll (hat 4 AAA Batterien) -> Level wurde auf 6,0V gesetzt testweise
- [x] Die RGB LED auf dem Controller soll nur für Statusrückmeldungen verwendet werden:
  - [x] Initialisierung angeschlossen, Controller bereit -> von gelb leuchten, dann grün permanent
  - [x] Duplo Hub connected -> blau 3 x blinken dann aus
  - [x] Button Stop gedrückt -> rot dauer blinken, wird aufgehoben bei erneutem Drücken auf Stop
  - [x] Button Record gedrückt: -> blau dauer blinken, aufgehoben bei erneutem Drücken auf Record
  - [x] Button Play gedrückt: -> gelbes dauer blinken bis Sequenz abgeschlossen oder Record erneut gedrückt wurde
  - [x] Bei Disconnect des Duplo Hubs 3 x weiß blinken
  - [x] Duplo Batterie Level low: 10x hektisches Blinken in rot alle 30 Sekunden
  - [ ] Duplo Color Sensor gibt bei reflektierenden Oberflächen manchmal kurz hintereinander unterschiedliche Farbewerte aus
    - [ ] Sensor muss "entprellt" werden
      - [ ] Sensor Events innerhalb einer kurzen Zeitspanne (< 250ms) ignorieren
      - [ ] Aufruf des Callbacks nur, wenn Sensor Event länger als Zeitspanne (5000ms) stabil ist

- [ ] Controller Sleep Modus einbauen
  - [ ] wenn keine GUI Aktion für mehr als 5 Minuten soll der Controller in den Sleep Modus gehen
    - [ ] Im Sleep Modus wird nur die Stop Taste überwacht, z.B. über Interrupt
    - [ ] Wird die Stop TAste aktiviert wacht der Controller auf und geht in den Normalbetrieb
    - [ ] Im Sleep Modus blinkt die RGB LED schwach blau alle 2 Minuten. Mal sehen wie lange das der Akku mitmacht

### 3. Code Optimierungen

- [ ] Optimierung des Codes
  - [ ] Redundanzen ausbauen
  - [x] Klare Verteilung der Verantwortlichkeiten:
    - [x] DuploHub -> Verbindung zur Hardware, Senden von Befehlen und Weiterleiten von Sensordaten
    - [x] myLegoHub ergänzt und erweitert die Legoino Lib und wird nur von DuploHub verwendet
    - [x] TrainController ist für die Umsetzung der GUI Kommandos und die Verarbeitung von Sensorrückmeldungen verantwortlich
  - [x] Reduzieren der Delays so dass die Initialisierung und die Herstellung der Bereitschaft des Controller deutlich schneller geht. Auch die Latenz bei Kommandos mus reduziert werden. Klären welche minimalen Delays notwenndig sind damit die BLE Kommandos sicher ankommen.

### 4. Hardware Modifikationen
- [x] Modifikation der Platine:
- [ ] Controller Batterie Level Low: hecktisches Dauerblinken in rot - erfordert Modifikation der Platine und Überwachung der Spannung durch MCU -> aktuell nicht realisierbar
  - [x] Stop Button wird direkt über einen Digital Pin eingelesen
  - [x] Neben den Analog Buttons LED, Sound, Play und Replay noch einen Biutton für das Pairing und Wasseraufnehmen einbauen. -_> Pairing scheint nicht sinnvoll, da zu kompliziert
    - [ ] Bei Pairing wird die ID des aktuell verbundenen Zuges gespeichert und primär beim nächsten Connect verwendet. Wird kein Hub mit der gespeicherten ID nach 60 Sekunden gefunden wird jeder aktive Hub verbunden
    - [ ] Pairing kann aufgehoben werden durch erneutes drücken des Knopfes
    - [ ] Bei Drücken von Wasseraufnehmen wird die gleiche Aktionssequenz wie bei dem blauen Duploplättchen durchgeführt

### 5. Dokumentation
- [ ] Dokumentation des gesamten Systems überarbeiten:
  - [ ] HW Architektur
  - [ ] SW Architektur
  - [ ] Build und Inbetriebnahme
  - [ ] Benutzerhandbuch
- [ ] Design eines kindgerechten Gehäuses für den Controller