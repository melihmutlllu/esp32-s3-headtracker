# Altium Files

Custom PCB source for the tracker board: TP4056 charge input, MCP1825S-SOT223
3.3 V LDO feeding the ESP32-S3 Mini and MPU9250, and a slide switch on the
battery line.

```text
headtracking_.PrjPcb   Project file
headtracking.SchDoc    Schematic
PCB1.PcbDoc             Board layout
Schlib1.SchLib          Custom schematic symbols (module footprints as blocks)
PcbLib2.PcbLib          Custom PCB footprints
```

Renders/exports of the current revision are in [`../images`](../images):
[schematic](../images/schematic.png), [3D view](../images/pcb-3d.png),
[routed layout](../images/pcb-layout.png).

Open `headtracking_.PrjPcb` in Altium Designer to edit. Commit these
source/design file types when present:

```text
*.PrjPcb
*.SchDoc
*.PcbDoc
*.SchLib
*.PcbLib
*.OutJob
*.BomDoc
```

Generated/local folders are excluded by `.gitignore` and should not be committed:

```text
History/
__Previews/
Project Outputs*/
```

Suggested release/export structure if you want to publish manufacturing files:

```text
hardware/outputs/gerber/
hardware/outputs/assembly/
hardware/outputs/bom/
hardware/outputs/pdf/
```

`hardware/outputs/` is ignored by default. For public releases, attach generated Gerber/NC drill/BOM/assembly files to a GitHub Release, or remove that ignore rule when you intentionally want them tracked.
