# Print and machining analysis

Two checks that ask whether a part can be made the way you mean to make it.

## Print analysis

**Inspect › Print Analysis** (`inspect.print`) looks at a body as a printer would, from a build
direction (+Z by default): whether the mesh is watertight and fits the build volume, the height
and layer count, the area of overhangs beyond the critical angle and the support they need (an
upper bound on the support volume), the thinnest wall and where it is, bridges and the longest
span, and an estimate of material and time from the filament, the flow rate, the layer height,
the infill and the walls. PLA, PLA+, PETG, ABS, ASA and TPU are built in, as are the common
infill patterns.

**Best Print Orientation** (`inspect.print_orient`) measures the same figures for the six axis
directions and ranks them, best first.

## Preparing a part for the printer

Three features on **3DEEE › Make** change the part so that what comes off the printer matches
what was drawn. Each is an ordinary feature in the timeline (a chamfer, a cut, a split), so it
follows the part when the part changes.

- **Elephant Foot** (`print.elephant_foot`) chamfers the edges that sit on the plate, 0.3 mm by
  default. The first layer is squashed wider than drawn; the chamfer undoes it, so the part
  stands on size. Give `up` when the part is printed in another direction.
- **Hole Compensation** (`print.hole_compensation`) finds every round bore (a cylindrical face
  with the void inside it) and cuts it 0.2 mm wider on the diameter. Bosses and shafts are left
  alone; `max_diameter_mm` leaves big bores alone too.
- **Split for Print** (`print.split`) cuts a body too big for the printer in two on a plane
  (Z through the middle by default) and sets alignment pins on the lower half with sockets in the
  upper, at the corners of the cut where both halves are solid. Pin diameter, length and the
  socket clearance are yours to set; `pins: false` leaves the halves plain.

```json
{"command": "print.split", "params": {"body": "Housing", "axis": "Z", "station_mm": 60,
 "pin_diameter_mm": 4, "pin_length_mm": 6, "clearance_mm": 0.3}}
```

## Printed stress

A part off a filament printer is not a block of plastic. It is a thin solid shell around a
lattice, bonded layer to layer, and analysing it as solid material overstates it, most of all
across the layers, which is where printed parts come apart.

**3DEEE › Make › Printed Stress** (`inspect.printed_stress`) takes what a slicer takes: the
filament, the infill pattern and density, the wall loops, the line width, the layer height and
the top and bottom layers. Those become an **effective orthotropic material**, different
stiffness and strength along the layers and across them, stacked along the build direction you
give, and the stress solver runs with it. Pick the body, the face it is held at and the face the
force acts on, type the force, and Compute.

The reply carries **two safety factors** and names which one governs:

- **within a layer**: von Mises against the in-plane strength,
- **between layers**: tension normal to the layers against the interlayer bond.

A part can sit well inside its yield stress and still delaminate. Bending a flat bar printed
lying down is governed within a layer; the same bar printed standing on end carries its bending
stress across the layers, and the verdict says so.

**Compare Infill** (`inspect.compare_infill`) runs the same part and load through every pattern
and tables mass, in-plane stiffness, deflection and both safety factors, with the stiffest
pattern and the safest between layers named. Every pattern weighs the same at a given density:
the pattern is a free choice of where the stiffness goes, not of how much material there is.
Grid, honeycomb and concentric run their walls straight up and carry the build direction well;
gyroid is the most even and the best in shear; lightning holds up the top skin and should not
carry load.

**Seeing the infill.** The solver does not mesh the lattice, so the infill has no geometry you
could section. **Infill on a Section** (`inspect.infill_section`) draws it instead: on a plane
through the part (a layer by default, at the body's middle) it lays down the strands a slicer
would, the pattern at its pitch and the wall loops around the outside, with gaps wherever the
part has a hole, as a 3D sketch. It is the lattice the numbers describe, not a slicer's toolpath.

What the model is, and is not: it is **homogenised**. Shell and infill are blended into one
material rather than meshed strand by strand, which is what makes a comparison cheap and fair,
and it is why the model will not find a stress concentration at one infill strand. The pattern
coefficients are representative engineering values. Interlayer strength is modelled at about
half of in-plane, the published band; it depends on nozzle temperature, which nothing here
knows, more than on anything set in the form. Walls are worth more than infill, and the model
agrees: four walls at 20 % beats two walls at 40 %, for less mass.

```json
{"command": "inspect.printed_stress", "params": {"body": "Bracket", "fixed": [0, 5, 5], "load": [80, 5, 5],
 "force_n": [0, 0, -20], "up": [0, 0, 1], "pattern": "Gyroid", "infill": 0.25, "walls": 3}}
```

The reply has `printed_material` (the moduli and strengths the solve used, the solid fraction
and the mass), `max_von_mises_mpa`, `max_layer_normal_mpa` and where they occur,
`safety_in_plane`, `safety_interlayer`, `governs`, `verdict` and `warnings`; with `"field": true`
also every cell's von Mises stress and its stress normal to the layers. `inspect.compare_infill`
takes the same parameters without `pattern` and answers with `patterns`, one row each, `stiffest`,
`safest_between_layers` and a `text` table.

## Machining analysis

**Inspect › Machining Analysis** (`inspect.machining`) checks a part against what a mill, a
drill and a lathe can do: internal corner radii against the cutter that would have to cut them,
pocket depth against width, thin walls against the material, holes against the standard drill
sizes (ISO 235 and the preferred series) and their depth against their diameter, undercuts and
regions no tool can reach, the setups the part would need, and for a turned part the shoulders
and grooves. Findings come back ranked by severity with the face they concern, and a verdict.
