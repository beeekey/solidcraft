# Lattices

**Lattice** (`construct.lattice`) fills a body with a lattice: a gyroid or another triply
periodic minimal surface, a strut lattice, or a honeycomb, under a solid skin if you want one.
It opens from **3DEEE › STRUCTURE**; pick the body, choose the family and cell, and **Build**
reports the volume fraction and advice next to the fields. It is the standard lightweighting construction of printed parts, and the heat-exchanger form
when the fill is a sheet TPMS, whose one continuous wall separates two interpenetrating channels.

The fill is described not by its boundary but by a scalar field over space, negative inside and
positive outside, as implicit modellers do it: a lattice is a trigonometric expression, a strut is
the distance to a segment, a skin is a shell of the body's own distance field, and the body's
interior clips all of it. There is no boolean to fail. At the end the field is meshed into a
closed triangle surface and comes back as a mesh body next to the source (`hide: true` hides the
source), which displays, measures, exports and prints like any other body. The reply carries the
volume fraction, the fill's volume, the triangle count and advice.

## What fills it

- `family: walled_tpms` thickens the mid-surface of the TPMS to `thickness_mm`.
- `family: solid_tpms` keeps the region below `level` (−1.4 to 1.4; 0 is half full).
- `family: beam` puts struts of `thickness_mm` along a cell's edges, rounded where they meet.
- `family: honeycomb` is a hexagonal honeycomb of `cell_mm` across the flats with walls
  `thickness_mm` thick, its cells running along `axis` (Z by default).

`cell` picks the surface (gyroid, schwarz_p, diamond, neovius, lidinoid, split_p, iwp) or the
strut cell (simple_cubic, body_centred, face_centred, octet, diamond, kelvin); `cell_mm` is the
period. `skin_mm` keeps a solid skin that thick inside the surface.

## Grading

`grading: along_axis` runs the thickness from `thickness_min_mm` at `grade_from` to
`thickness_max_mm` at `grade_to`; `from_point` grows it with the distance from `grade_from` out to
`grade_radius_mm`. `by_stress` runs the stress analysis first (the same `fixed`, `load`, `force_n`
and `resolution` as `inspect.stress`) and puts the thickness where the von Mises stress is: thin
walls where nothing happens, the thickest where the part works hardest. The reply then also says
how dense the fill came out near the clamp and near the load.

## Samples

The field is sampled on a grid of `samples_per_cell` points per cell along each axis (8 by
default; 16 is smooth and eight times the work). A wall thinner than two samples comes out
ragged or vanishes, and the advice says so; so does a cell bigger than the body's thinnest side.
A fill that would need more than four million samples is coarsened to fit and says by how much.

## From a script or an agent

```json
{"command": "construct.lattice", "params": {"body": "Bracket", "family": "walled_tpms",
 "cell": "gyroid", "cell_mm": 8, "thickness_mm": 1.2, "skin_mm": 1.5, "hide": true}}
```

returns the volume fraction (about a third of the bracket at these numbers), the fill's volume
in mm³ and the new body's name.

## Variable shell and blend boolean

The same distance fields give two more tools on **3DEEE › Structure**. **Variable Shell**
(`construct.variable_shell`) hollows a body to a skin, or grows it (`operation: offset`), by a
thickness that is uniform, graded along an axis (`grading: along_axis`, `grade_from`, `grade_to`,
`thickness_min_mm`, `thickness_max_mm`) or graded from a point. **Blend Boolean**
(`construct.blend_boolean`) joins, cuts or intersects a body with a tool body on their distance
fields, with the seam rounded over `blend_mm`: a fillet where two castings meet, without a kernel
fillet having to find the edge. Both come back as mesh bodies; `sample_mm` sets the field's
spacing (about 60 to 80 samples along the longest side by default) and the reply says the volume.
