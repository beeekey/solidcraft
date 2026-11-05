# Stress analysis

**Inspect › Stress Analysis** (`inspect.stress`) answers the question every part eventually gets
asked: held here and pushed there, how far does it move, how hard is it stressed, and does it
hold?

Pick the body, the face it is held at and the face the load acts on, type the force in newtons
(three components, in the world's axes) and press **Compute**. The dialog reports the largest
displacement, the peak von Mises stress and, when the material's yield stress is known, the
safety factor and a one-line verdict. Over the model, every cell of the solver's mesh is coloured
by its stress on the Curvature Map's scale (so it follows the theme and its colour-vision
modes), from nothing to the peak, with a colour bar in the dialog.

## How it is solved

SolidCraft has its own finite-element solver (`crates/analysis`); nothing is sent anywhere and no
other program is needed. The body is voxelised on a regular grid, each cell becomes a trilinear
hexahedral element of linear elasticity, the held side is clamped, the force is spread over the
loaded side, and the system is solved by preconditioned conjugate gradients. The solver is held
to closed-form elasticity in its tests: Hooke's law on a pulled rod, Euler–Bernoulli with the
shear term on a cantilever.

**Resolution** is the number of elements along the body's longest side (6 to 60, 24 by default).
Bending is carried by the elements through the thickness, and a mesh two or three elements thick
reports a fraction of the real stress while converging perfectly, so the answer says so: when
fewer than six elements span the thinnest wall, a warning names the resolution that would fix it.
Read the peak together with that advice.

One thing linear elasticity cannot do: a clamp and a point load are stress singularities, so the
peak right at them grows as the mesh is refined. Read the peak with the mesh advice.

## Buckling and collapse

A slender part in compression folds long before its material yields, and a stress solve says
nothing about it. Tick **Buckling** (or pass `buckling: true`) and the solver also finds the load
factor at which the stiffness, softened by the stress the part already carries, stops being
positive definite: applied load × factor is the load a perfect member buckles at. Because no
member is perfect, it also reports how much a bow of L/500 (the straightness a rolled section is
sold to) grows at this load, 1 / (1 − 1/factor), and the bending stress that adds. The solver is
held to Euler's column in its tests.

**Follow to collapse** then bows the member into its buckling mode and walks the load on with a
co-rotational large-displacement solve until no equilibrium is left. That load, always below the
eigenvalue, is the one a real member folds at. It costs a nonlinear solve per load step.

## Material

The elastic constants come from the body's physical material through a small built-in handbook
(steel, stainless steel, aluminium, brass, copper, titanium, cast iron, ABS, PLA, nylon,
polycarbonate, oak, glass; the default material counts as a generic plastic). The dialog shows
which was assumed. From a script or MCP, `young_modulus_pa`, `poisson_ratio` and `yield_pa`
override it.

## Options

- **Own weight** adds gravity (9.81 m/s² along −Z) from the material's density.
- **Resting support** solves the mount as one that only pushes back, like a foot on a table
  rather than a bolt through a flange: nodes the support would have to pull on are released and
  the solve is repeated until the set settles. The result counts the nodes that lifted.

## Several bodies

Pick two or more bodies that touch (from a script, `bodies: [names]`) and the solver puts them
on one grid. **Bonded**, the default, shares the nodes across every interface: what a weld or a
glue line does, and what two features of one part are anyway. **Sliding contact** and **Contact
with friction** instead duplicate the nodes where the bodies meet and link them with a contact
that pushes but never pulls: the parts lift apart where the load opens the joint, slide freely or
hold within the friction coefficient, and a preload (`preload_n`, what a bolt does) clamps the
interfaces shut. The answer counts the links, how many opened and how many slid. A body that
ends up held by nothing (every contact under it opened) has no static answer, and the command
says so instead of printing one. Each body takes the stiffness of its own material, relative to
the first body's; a modulus given outright applies to them all.

## Thermal analysis

**Inspect › Thermal Analysis** (`inspect.thermal`) answers the other question a part gets asked:
how hot does it run. Pick the body, the face the power goes in over and, if there is one, the
face held cold (a cold plate, a massive mount), type the watts and press **Compute**. The part is
solved in still air: steady conduction through the same voxel grid, each exposed face losing
heat by the free-convection correlation for its orientation (Churchill–Chu for a wall, McAdams
for a face looking up or down, Elenbaas for a channel between fins) and by radiation at the
material's emissivity. The dialog reports the peak and mean surface temperatures, how the heat
left and how finned the part is, and paints the cells from the air's temperature to the peak.
It knows that fins too close together choke, and it can put the part in a sealed box
(`enclosure_area_m2`) whose air then runs above the room. It is not a flow solve: it assumes
still, unbounded air unless a box is given.

The thermal constants come from the handbook too (conductivity and a plain-surface emissivity
per material); `conductivity_w_mk` and `emissivity` override them.

## The hot spot, a point, and the deck

Three small tools on **3DEEE › Generative** read the same solve. **Stress Zone**
(`inspect.stress_zone`) builds the cells above a share of the peak von Mises stress (half by
default) as a mesh body of their own: where a fillet or a rib belongs. **Probe Result**
(`inspect.probe`) reads the field at a point you click. **Export FEM Case** (`inspect.export_fem`)
writes the voxel mesh, the held nodes, the load spread over the loaded nodes and the material as
a complete CalculiX deck (`case.inp`, with `solve.sh`), for a second opinion from another solver.

## Printed parts

A part that will be 3D printed is a shell around a lattice, weaker between its layers than
within them. **3DEEE › Make › Printed Stress** runs this same solver with the material the print
settings amount to, stacked along the build direction, and judges the answer within a layer and
between layers; **Compare Infill** tables every pattern. See
[Print and machining analysis](manufacturing.md#printed-stress).

## Flow analysis

**Inspect › Flow Analysis** (`inspect.flow`) puts the part in a fluid: through its passages
(pick a face where the coolant enters and one where it leaves) or around it in a tunnel. A
lattice Boltzmann solver on the same voxel grid settles the flow and reports the Reynolds
number, the pressure drop, the flow rate, the peak velocity and the drag on the part. With
**Heat from the part** on, a steady conjugate heat solve follows: conduction through the metal
and convection into the fluid as one system, so the dialog says what temperature the fluid
leaves at, how many of the part's watts it carries and how hot the part runs. Water, air, oil
and glycol are built in. Everything is laminar; past what the grid resolves the solver carries
the unresolved motion as an eddy viscosity and the warnings say the pressure drop is indicative.

## From a script or an agent

```json
{"command": "inspect.stress", "params": {"body": "Bracket", "fixed": [0, 5, 5], "load": [100, 5, 5],
 "force_n": [0, 0, -100], "young_modulus_pa": 210e9, "resolution": 30}}
```

The reply carries `max_displacement_mm`, `load_displacement_mm`, `max_von_mises_mpa` and where
they occur, `safety_factor` when a yield stress is known, `mesh_advice`, `verdict` and
`warnings`. With `"field": true` it also returns every material cell's stress, which is what the
dialog paints. The command changes nothing in the design.
