## Aktuelle ToDos

### 1. Record /Replay Funktion:
- [ ] Die Replay Liste wird nicht nur genau einmal abgefahren sondern in einer Schleife -> Sollverhalten: Liste einmal abfahren und dann den Replay Modus beenden
- [ ] Während Replay müssen alle Kommandos außer Beenden des Replaymodus und Stop ignoriert werden
- [ ] Ebenso sollten die  über den Color Sensor identifizierten Aktionen im Replay Modus NICHT ausgeführt werden
- [ ] Im Record Modus jedoch sollen die über den Color Sensor identifizierten Befehle mit aufgezeichnet werden
- [ ] Das Timing der aufgezeichneten Befehle muss überprüft werden und sollte bis auf 500ms mit der Aufgezeichneten Sequenz beim Abspielen übereinstimmen

### 2. Sonstige Softwareänderungen
- [ ] Der Voltage Sensor muss aktiviert werden um die Spannung der Duplobatterie zu überwachen. Klären bei welchem Soannungslevel eine Warnung ausgegeben werden soll (hat 4 AAA Batterien)
- [ ] Die RGB LED auf dem Controller soll nur für Statusrückmeldungen verwendet werden:
  - [ ] Initialisierung angeschlossen, Controller bereit -> von gelb blinken, dann grün permanent
  - [ ] Duplo Hub connected -> blau 3 x blinken dann aus
  - [ ] Button Stop gedrückt -> rot dauer blinken, wird aufgehoben bei erneutem Drücken auf Stop
  - [ ] Button Record gedrückt: -> blau dauer blinken, aufgehoben bei erneutem Drücken auf Record
  - [ ] Button Play gedrückt: -> gelbes dauer blinken bis Sequenz abgeschlossen oder Record erneut gedrückt wurde
  - [ ] Bei Disconnect des Duplo Hubs 3 x weiß blinken
  - [ ] Duplo Batterie Level low: hektisches Dauerblinken in blau
  - [ ] Controller Betterie Level Low: hecktisches Dauerblinken in rot
- [ ] Controller Sleep Modus einbauen
  - [ ] wenn keine GUI Aktion für mehr als 5 Minuten soll der Controller in den Sleep Modus gehen
    - [ ] Im Sleep Modus wird nur die Stop Taste überwacht, z.B. über Interrupt
    - [ ] Wird die Stop TAste aktiviert wacht der Controller auf und geht in den Normalbetrieb
    - [ ] Im Sleep Modus blinkt die RGB LED schwach blau alle 2 Minuten. Mal sehen wie lange das der Akku mitmacht

### 3. Code Optimierungen

- [ ] Optimierung des Codes
  - [ ] Redundanzen ausbauen
  - [ ] Klare Verteilung der Verantwortlichkeiten:
    - [ ] DuploHub -> Verbindung zur HArdware, Senden von Befehlen und Weiterleiten von Sensordaten
    - [ ] myLegoHub ergänzt und erweitert die Legoino Lib und wird nur von DuploHub verwendet
    - [ ] TrainController ist für die Umsetzung der GUI Kommandos und die Verarbeitung von Sensorrückmeldungen verantwortlich
  - [ ] Reduzieren der Delays so dass die Initialisierung und die Herstellung der Bereitschaft des Controller deutlich schneller geht. Auch die LAtenz bei Kommandos mus reduziert werden. Klären welche minimalen Delays notwenndig sind damit die BLE Kommandos sicher ankommen.

### 4. Hardware Modifikationen
- [ ] Modifikation der Platine:
  - [ ] Stop Button wird direkt über einen Digital Pin eingelesen
  - [ ] Neben den Analog Buttons LED, Sound, Play und Replay noch einen Biutton für das PAiring und Wasseraufnehmen einbauen
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