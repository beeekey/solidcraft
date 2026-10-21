# Generative design

Every other analysis answers *is this part good enough*. Generative design answers a different
question: given where the part is held, what pushes on it, and how much room there is, **what
shape should it be?** You give the boundary conditions; the search invents everything between
them, scores each attempt with the finite-element solver, and offers several finished designs to
choose between. Each becomes a mesh body in the design.

## Commands

All three open from **3DEEE › GENERATIVE**. Pick the faces the part is held at and the faces it
is pushed on, type the force, and pick the body that is the design space (or the body to check);
with no faces picked, the bracket already described in the form (held on a wall at the left,
pushed down at the right) is used. **Run** searches and builds, and the designs come back as rows
in the dialog.

- **`generative.study`** ("Generative Design"): the search. It works the way a
  designer sketches, with lines and plates between the places that matter. A candidate is a
  skeleton of struts of some radius between anchor points and free interior nodes, plus
  triangular plates where a web carries the load better than a bar. Candidates are mutated and
  recombined over generations; each is first scored by pin-jointed statics on the skeleton (a
  fraction of a millisecond, enough to throw away mechanisms) and the survivors by voxel finite
  elements, which see the joints, the plates and the bending. The answer is several designs,
  best first and deliberately different from each other, each with its mass, safety factor and
  verdict; `build: n` adds design n as a mesh body.
- **`generative.free_form`** ("Free-Form Body"): skips the search. The density method (SIMP)
  does not need a layout: given only where the part is held, what pushes on it and how much
  room there is, it grows a body as a density field over the design space, free to become a
  shell, a web that thickens where it is working hard, or a smoothly blended junction, none of
  which a skeleton can describe. With `meet_target` it keeps giving the field more volume until
  the safety factor is reached.
- **`generative.check`** ("Check Design"): the checks a person does on the chosen design, on an
  existing body: a finer mesh than the search used, buckling of slender members, fatigue life by
  Basquin's line with a Goodman mean-stress correction, the Marin surface and size factors for
  the finish the part will have, damage by Miner's rule over the load cases' cycle counts, and
  whether the thing can be made at all (overhangs needing support, faces no mould could release,
  surfaces no tool can reach).
- **`inspect.fibre`** ("Grain Failure"): the stress check in the frame of a material's own grain,
  for bamboo, wood and unidirectional laminates. Von Mises is the wrong question for a material
  with a grain; this one compares the stresses along and across the fibre with the strengths
  that direction has (Tonkin, Moso and Guadua bamboo are built in).

## Regions

Mounts and loads are faces of an existing body (`{face, point}`, taken as a slab a few
millimetres thick) or boxes in millimetres. A mount is bonded by default; set `bonded: false`
for a foot that merely rests on something and can push back but never pull. A load has a force
in newtons and, for the heat goals, watts; `cycles` and `stress_ratio` describe a load that
repeats, which is what turns the fatigue factor into a life. Two load cases are two answers, and
the worse one governs, never their sum.

## Goals

`carry_load` (default) carries the loads with as little material as possible. `remove_heat` gets
the heat out of the loaded areas into a coolant stream, scored by conduction with a convective
film and verified by the flow solver. `low_drag` carries the loads while disturbing the stream
as little as possible. The stream runs straight through the design space along one axis.

Gravity is on by default: a part big enough for its own weight to matter has to hold itself up
before it holds anything else, and forgetting it is the expensive direction to be wrong in.

## Topology optimisation

**3DEEE › Generative › Topology Optimize** (`construct.topology_optimize`) is the direct route
when the part already exists: pick the body, the face it is held at and the face the force acts
on, type the force and the fraction of material to keep. The stress solver runs on a voxel grid
and the most-worked cells of every section along the load path are kept, always connected from
the mount to the load, floating islands dropped. The kept cells come back as a new mesh body
with their corners eased; the source stays (or is hidden with `hide: true`). The reply says what
fraction was kept and how coarse the grid was; raise `resolution` for finer struts.

## Limits

The search scores hundreds of candidates on a coarse grid, so its numbers are relative; the
check runs the chosen design properly. Bodies come back as meshes, not B-rep solids: they
measure, render and export, and `generative.check` and the stress and thermal analyses take
them, but solid features need a B-rep body. Geometry is extracted from the voxel grid by
marching cubes with a few smoothing passes.
