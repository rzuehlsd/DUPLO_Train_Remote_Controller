# Hardware Assets

This directory now contains the complete KiCad design files for the **DuploTrainController NG** handheld controller together with the generated Gerber fabrication outputs.  The top-level structure is:

```
hardware/
├── Case/                       # Enclosure or mechanical assets (empty placeholder for now)
├── DuploTrainController NG/    # KiCad project directory
│   ├── DuploTrainController NG.kicad_pro
│   ├── DuploTrainController NG.kicad_pcb
│   ├── DuploTrainController NG.kicad_sch
│   ├── DuploTrainController NG.kicad_prl
│   ├── DuploTrainController NG.kicad_pcb.zip
│   ├── 5 Buttons an ADC.xlsx
│   ├── Gerber/                 # Fabrication outputs
│   └── fp-info-cache           # Footprint cache
└── README.md                   # This document
```

Open the project in KiCad using `DuploTrainController NG.kicad_pro`.  The spreadsheet documents the ADC ladder calculations for the push buttons.  The `Gerber/` folder contains ready-to-send manufacturing files that match the current PCB revision.

> **Note:** Large binary and generated files are intentionally stored in version control so collaborators have an exact copy of the board as fabricated.  If you regenerate outputs, please audit the diffs before committing to avoid unnecessary churn.
