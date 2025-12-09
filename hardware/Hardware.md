# Duplo Train Controller Hardware

**Author:** Ralf Zühlsdorff  
**Copyright:** (c) 2025 Ralf Zühlsdorff  
**License:** MIT License

---

The hardware package covers both the custom ESP32-S3 controller PCB and the printable enclosure that transforms the electronics into a DUPLO-friendly handheld remote. Everything required to reproduce or iterate on the physical controller—KiCad sources, manufacturing files, OpenSCAD models, and helper assets—lives in this directory.

## Directory Layout

```
hardware/
├── Case/                       # OpenSCAD sources and library dependencies for the enclosure
├── DuploTrainController NG/    # KiCad PCB project and manufacturing outputs
└── README.md
```

The top-level `README.md` links back to the main project documentation and provides quick navigation hints for contributors.

## PCB Development

- The PCB is captured in KiCad 8 using `DuploTrainController NG.kicad_pro` as the entry point.
- A Tenstar ESP32-S3 SuperMini module forms the core, surrounded by button ladder, encoder, power management, and RGB LED circuitry sized for DUPLO ergonomics.
- `5 Buttons an ADC.xlsx` explains the resistor ladder math that multiplexes six push buttons into a single ADC input.
- Fabrication assets live under `Gerber/` and the zipped board archive `DuploTrainController NG.kicad_pcb.zip`. Review diffs carefully before committing regenerated output to avoid noise.
- Footprint cache data is retained in `fp-info-cache` so collaborators can rebuild without missing library references.

```
DuploTrainController NG/
├── DuploTrainController NG.kicad_pro   # KiCad project file (open this first)
├── DuploTrainController NG.kicad_sch   # Schematic for the controller PCB
├── DuploTrainController NG.kicad_pcb   # Routed PCB layout
├── DuploTrainController NG.kicad_prl   # Project-specific settings and plot config
├── DuploTrainController NG.kicad_pcb.zip
├── DuploTrainController NG-backups/    # Timestamped KiCad backups
│   └── DuploTrainController NG-*.zip
├── 5 Buttons an ADC.xlsx               # Button ladder calculations
├── Gerber/
│   ├── DuploTrainController NG-F_Cu.gbr
│   ├── DuploTrainController NG-Edge_Cuts.gbr
│   ├── DuploTrainController NG.drl
│   ├── DuploTrainController NG-job.gbrjob
│   └── GCode/
│       ├── F-CU.nc
│       ├── drill.nc
│       └── cutout.nc
└── fp-info-cache                       # Local footprint cache for reproducible builds
```

<p align="center">
	<img src="../images/ControllerBoard%20Schematic.png" alt="Controller board schematic" width="78%">
	<img src="../images/ControllerBoard%20PCB.png" alt="Controller board PCB layout" width="18%">
</p>

The 6 function buttons are handled via ADC input with ADCButton lib, so just one button at a time can be detected. The Excel sheet is used to calculate the resistors so that voltage difference between two steps is in the same range.

The encoder button is detected via a different GPIO pin for two reasons. First it provides a kind of second level for all function switches to implement future requirements. But main reason was that this button is used to wake the controller from deep sleep.

The Lithium 3.7V batterie is connected to B+ and B- at the ESP32-S3. The schematic and pcb could be kept very simple as all functionality to handle charging is handled by the ESP32-S3 Super Mini. 

PCB has been produced by PCBways.


## Case Development

- The enclosure is modelled in OpenSCAD inside `Case/Controller.scad`, assembling MachineBlocks primitives to match DUPLO dimensions.
- `Case/lib/` vendors the MachineBlocks modules to make the CAD fully reproducible without external dependencies.
- Parametric dimensions cover battery cavity, button spacing, and encoder fit, enabling quick iteration for future PCB tweaks.
- Export STL files from OpenSCAD before printing; small tolerances are already tuned for typical FDM printers.

```
Case/
├── Controller.scad            # Top-level OpenSCAD model for the enclosure
├── MachineBlocks/             # Upstream MachineBlocks sources (see vendor README)
└── lib/                       # Minimal subset of MachineBlocks modules used here
	├── base.scad
	├── block.scad
	├── connectors.scad
	├── duo.scad
	├── pcb.scad
	├── plug.scad
	├── shapes.scad
	├── socket.scad
	├── svg3d.scad
	└── text3d.scad
```
THe case was designed based on a lib and an example from MachinBlocks. 
It covers the case with a bottom part and a lid with cutouts for 6 buttons, the encoder and the usb connector.

There are 2 black buttons for recording (with red dot) and replay (with white triangle). The red button is used to initiate the emergency stop, the blue buttond is used for water refill, the white button to switch the headlight led of the train and the yellow to activate different sounds.


<p align="center">
	<img src="../images/ControllerCase.png" alt="Controller Case" width="78%">
</p>

The design is provided as scad file and exports to 3mf and stl files have been added for 3d printing.


> **Note:** Both electrical and mechanical assets are version-controlled to keep the manufactured controller in sync with the firmware. When regenerating Gerbers or STLs, audit changes to ensure they reflect intentional design updates.
