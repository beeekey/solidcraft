# Mechanical generators

The `design.*` commands build the parts a machine is made of from their catalogue numbers. Each
one goes into its own new component, drawn with ordinary sketches and features, so the result is
as editable as anything drawn by hand: the timeline shows the sketch and the extrude, and the
component can be moved, joined and patterned like any other. The reply names the component and
the numbers the part was built to.

Every generator has a button on the **3DEEE** tab (GEARS, DRIVE, PARTS, STRUCTURE). Its form
opens with values that build a part; change what you like and press OK. **Frame** takes the
lines of a sketch you pick as its members and **Pipe Route** the lines joined end to end as its
waypoints, so a frame or a pipe run is drawn as a sketch first; with nothing picked, the members
or waypoints typed in the form are used. The same parameters go
to the command from the palette, a script or the agent.

All of them take millimetres and return millimetres. Where a standard fixes a number (a belt's
pitch, a chain's roller, a NEMA frame's bolt square) the command knows it and reports it.

## Gears

- **Spur gear** (`design.spur_gear`): `module`, `teeth`, `width`, and `pressure_angle_deg`
  (20 by default). The flanks are true involutes of the base circle, fitted as splines, closed by
  tip and root arcs. `bore_mm` puts a hole through it; `internal: true` makes a ring gear with
  the teeth pointing inward and `rim_mm` of rim. `helix_deg` makes it helical and
  `herringbone: true` two opposite helices meeting at the middle. A helical flank is a helicoid,
  which the kernel reaches only through minutes of lofting, so helical and herringbone gears are
  built as mesh bodies (the outline turned station by station, with the bore) unless
  `exact: true` asks for the lofted B-rep and accepts the wait.
- **Gear rack** (`design.gear_rack`): the straight counterpart, `teeth` of them along X with
  `back_mm` under the roots.
- **Bevel gear** (`design.bevel_gear`): the outline swept to the cone's apex, `pitch_angle_deg`
  being the cone's half angle (45 for a mitre pair) and `face_width` the length along the cone.

- **Worm** (`design.worm`) and **worm wheel** (`design.worm_wheel`): the driving screw of a worm
  drive, a trapezoidal thread of `module` and `starts` on a cylinder of `pitch_diameter` (its lead
  `starts × π m`, the lead angle reported), and the wheel it drives, with helical teeth at the
  worm's lead angle and the rim throated so the tips follow the worm (the throat arc's radius is the
  worm's pitch radius less a module; a face wider than the worm it wraps is refused). Both are mesh
  bodies, a helicoid being what the kernel takes minutes over. The ratio is teeth over starts.
- **Crown gear** (`design.crown_gear`): teeth on the flat face of a disc, the limit of a bevel at a
  pitch angle of 90°: radial grooves of constant width, narrowing with depth at the pressure angle,
  on a band `face_width` wide centred on the pitch circle, driven by a spur pinion of the same
  module at right angles. A mesh body.

## Shafts and springs

- **Shaft** (`design.shaft`): `sections` chained along +Z, each with a `diameter` and `length`,
  optionally a `keyway` or a circlip `groove` in its middle; `chamfer_mm` on the end faces.
- **Spring** (`design.spring`): `kind` compression (with closed end coils, its pitch and spring
  rate reported), extension (with hooks swept at both ends) or torsion (with straight legs),
  from the wire, mean diameter, active coils and free length. The end coils, hooks and legs are
  separate bodies in the spring's component, touching the coil rather than fused to it: a boolean
  between two helical faces costs the kernel minutes, and the spring weighs and looks the same.

## Belts and chains

- **Pulley** (`design.pulley`): a timing pulley for a `standard` (GT2-2M, GT2-3M, GT2-5M, HTD-3M,
  HTD-5M, HTD-8M, HTD-14M, T2.5, T5, T10, AT5, AT10, MXL, XL, L, H) with `teeth` and `width`.
  The groove form follows the standard: round for HTD, straight flanks for T and the inch belts,
  the deeper rounded groove for GT. `bore_mm` with `bore_style` plain, keyway or set_screw;
  `flanges: true` adds flanges; `plain_roller: true` makes a smooth deflection roller instead.
- **Belt** (`design.belt`): the loop over `wheels`, each a toothed pulley (`teeth`) or a plain
  roller the belt's back runs over (`roller_diameter_mm`), with `side: -1` for an idler pressed in
  from outside. The command solves the tangent spans and wraps for the cord line of the standard
  (the pitch line differential of a toothed pulley, the belt's back on a roller) and reports the
  length, the standard belt it matches (exact when it lands on a whole tooth), the wrap and the
  teeth in mesh on each pulley, and the clearance between spans. `build: false` only measures;
  otherwise the belt is swept as a solid of the standard's thickness. A loop that crosses itself,
  or wheels that overlap, is refused unless `allow_crossing` is set for a crossed flat belt.
- **Sprocket** (`design.sprocket`) and **chain** (`design.chain`) for roller chains 05B, 06B,
  08B, 10B, 12B and ANSI 25, 35, 40, 50, 60: the sprocket's pitch, tip and root diameters and
  tooth width follow ISO 606; the chain reports the links a centre distance needs (rounded up to
  even) and the exact centre distance for a whole number of links, and builds the rollers and
  alternating inner and outer plates around the loop.

## Profiles and motors

- **T-slot profile** (`design.tslot`): a 2020, 3030 or 4040 aluminium extrusion of any `length`
  along Z with its four slots and the centre bore.
- **Stepper motor** (`design.stepper_motor`): a NEMA 8, 11, 14, 17 or 23 frame with the pilot
  boss and the shaft (with its D flat) along +Z from the front face at z = 0, and the bolt square
  and bolt size in the reply for the bracket that holds it.

## Frames and pipes

- **Frame** (`design.frame`): structural members along lines. `members` lists each one's `from` and
  `to`; `profile` names the family (square, rectangular or round tube, round rod, angle, channel,
  I-beam or a T-slot extrusion) with a catalogue `size` ("40x40x3", "UNP 50", "IPE 80", "2020") or
  the dimensions outright, centred on the line (`offset` shifts it, `rotation_deg` turns it). Where
  exactly two members meet at an end they are mitred on the bisecting plane (`ends: butt` leaves
  every member square), and a member that ends on another member's length is trimmed back to its
  face. Every member is a body of its own, extruded from a sketch of the section, so it stays
  editable; the reply counts the joints and gives the total length and the section's area.
- **Pipe route** (`design.pipe_route`): a tube of `outer_diameter` and `wall` along `points`, with
  tangent bends of `bend_radius` at every corner, as a Pipe feature on a 3D sketch of lines and
  arcs. A bend that does not fit between its neighbours is refused by its corner; the reply has the
  centreline length.

- **Fillet weld** (`design.fillet_weld`): a bead along an inside corner of a body, a right
  triangle with `leg_mm` legs on the two faces that meet at the picked edge, swept along it,
  straight or round a pipe's foot; the reply has the length, the throat and the bead's volume.
  An outside corner is refused: there is nothing to fill.

## Plastic details

- **Snap hook** (`design.snap_hook`): a cantilever arm with a square catch face and an engagement
  ramp, from `arm_length`, `thickness`, `hook_depth`, `hook_length` and `width`.
- **Heat-set insert boss** (`design.insert_boss`): a boss for an M2 to M6 brass insert, the hole
  and boss diameters and the bore depth from the usual insert sizes, refused when the boss is too
  short for the insert.

## Bamboo

**Bamboo culm** (`design.bamboo_culm`): a tapered tube from the butt to the tip with the wall
thinning along it, node collars standing proud at the node spacing and diaphragms closing the
bore at each node. Pair it with `inspect.fibre`, which checks a grained material in its own frame
with the bamboo species built in.

## Construction kit

**Brickify** (`construct.brickify`) replaces a body with parts off the shelf, from the systems you
pick in the dialog: the body is laid out on a stud grid and filled with interlocking bricks, layer
by layer with the orientation alternating; long straight runs become T-slot profiles; tall columns
get a threaded rod through them and the shorter ones a hex bolt, each with its nuts and washers.
The brick system is classic bricks (8 mm studs, 9.6 mm high), classic plates (3.2 mm thin) or
large bricks (16 mm studs); the profile series is 2020, 3030 or 4040; rods are M6 to M12 and
bolts M3 to M8. The reply is the bill of materials with counts, the coverage, and the kit as a
mesh body next to the original (`hide: true` hides the source). `bricks`, `profiles`, `rods` and
`bolts` switch each family off. A profile needs its own section inside the body and a straight run of
`min_profile_mm` (default 40, two 2020 cells); rods clamp columns of four layers, bolts of two, on a
lattice every six studs centred on the body. A family that is on but finds nothing says why in the
warnings, so a part thinner than a profile comes back as bricks alone.

## From a script or an agent

```json
{"command": "design.belt", "params": {"standard": "GT2-2M", "width": 6,
 "wheels": [{"centre": [0, 0], "teeth": 20}, {"centre": [100, 0], "teeth": 20}], "build": false}}
```

returns the loop length (240 mm, a 120-tooth belt), the wraps and the teeth in mesh, and changes
nothing until `build` is left on.

## The bamboo workshop

Designing in bamboo is a different job: the joint fails before the tube, not all of a frame can
be bamboo, and the culm is shorter than its axis because it disappears into a connector at each
end. Four generators on **3DEEE › Parts** and **3DEEE › Drive** carry that:

- **Bamboo Culm** (`design.bamboo_culm`): tapered, noded, with its wall volume.
- **Bamboo Connector** (`design.bamboo_connector`): a Muffe, a hub with a glued sleeve per culm.
  Each bore is the culm's diameter plus a glue gap all round (0.3 mm by default: a joint
  squeezed metal to metal is a starved joint) and runs in by exactly the engagement, so the
  culm bottoms out on hub material. The reply carries the glued area, which **Bamboo Joint**
  turns into a capacity.
- **Bamboo Frame** (`design.bamboo_frame`): a diamond bicycle frame from the numbers on a frame
  drawing (seat tube and angle, top tube, head tube and angle, chain stay, bottom bracket drop,
  dropout spacing; origin at the bottom bracket, X forward, Z up) or a box trailer (chassis,
  cross members, drawbar). Culms on the bamboo axes, a metal head tube, a connector at every
  junction, and the cut list: each member's axis length less the two hubs it reaches between.
  Chain stay length is centre-plane, so the stay itself runs a little longer to its dropout.
- **Bearing** (`design.bearing`) and **Damper** (`design.damper`): the hardware a frame hangs on,
  drawn to the dimensions that have to fit (ISO 15 boundary dimensions for the bearings, with
  their catalogue ratings; eye to eye, stroke, body and eye bore for the damper). **Bearing
  Life** and **Damping and Isolation** on the CALCULATE panel take their numbers.

## Editing a frame

Once a frame stands, **3DEEE › Structure** edits a member from its own geometry (its axis is its
longest straight edge, so any straight bar qualifies): **End Cap** joins a plate over an end;
**Mitre** cuts two members on the plane halving their corner so the pair closes; **Notch** cuts one member back around another and keeps the cutter; **Change Section** rebuilds
the member on its axis with a new profile and hides the old one; **Extend Member** pushes an end
out or trims it back; **Groove Weld** fills the gap between two parallel faces of two bodies as
the weld's bead, a body of its own with its volume in the reply. **Fillet Weld** stays on the
same panel.
