# Design calculators

A set of checks that take numbers rather than geometry: the hand calculations a mechanical
engineer does beside the model. Each is a command, so it runs from the command box, a script and
the MCP server, and each returns the figures it used, a one-line `verdict` and `warnings`. Each
also has a dialog under **3DEEE › CALCULATE**: the fields with sensible defaults, a **Compute** button, and
the reply laid out as rows with the verdict in bold, the advice and the warnings beneath. A
calculator changes nothing in the design, so the dialog has Close rather than OK.

| Command | What it answers |
|---|---|
| `inspect.bolted_joint` | A bolted joint as two springs in parallel: the bolt's stiffness over its grip and the clamped parts' stiffness from the squeezed frustum. Give the size (M3 to M36), grade (4.6 to 12.9, A2-70), the clamped length and the loads, and it reports the preload, how the external load splits between bolt and clamp, where the joint separates, whether it slips, the tightening torque and the fatigue safety of the thread. |
| `inspect.thread_engagement` | How many diameters of thread a screw needs in steel, aluminium or plastic, and whether the engagement given is enough. |
| `inspect.bearing_life` | ISO 281 L10 life of a catalogue bearing (deep-groove, thin-section, bottom-bracket and headset sizes) or of any bearing by its C and C0, in revolutions, hours and kilometres, with the static safety. |
| `inspect.damping` | A mass on a spring with a damper or a material's loss factor: natural frequency, damping ratio, how many cycles a knock takes to die down, and the frequency above which the mount isolates rather than amplifies. |
| `inspect.fit` | A hole class and a shaft class at a nominal size (ISO 286: `H7/g6`): the limits, the clearance or interference range, and for an interference fit with the hub geometry given, the contact pressure, the force to press it on, the torque it holds and the hub's hoop stress. |
| `inspect.general_tolerance` | What an untoleranced dimension is allowed under ISO 2768 (f, m, c, v, or a drawing note such as `ISO 2768-mK`): lengths, radii and chamfers, angles. |
| `inspect.tolerance_stack` | A chain of dimensions with tolerances: the gap's worst case and its statistical (RSS) range, the contributors ranked by their share, and whether a required minimum and maximum are met. |
| `inspect.hole_stack` | From a clearance hole as drilled and the grip under the head: the screw it was drilled for (ISO 273 series), the next listed ISO 4762 length with washer and nut, and the drill, counterbore and countersink sizes that go with it. |
| `inspect.bamboo_joint` | A culm glued into a socket, checked the three ways it fails. The culm shears out a plug along its own fibres (bamboo's interlaminar shear is about 5 MPa, a thousandth of its tensile strength, so unless the epoxy is worse the bamboo governs and a better glue changes nothing); the glue line lets go; or the sleeve crushes the culm oval under the bending moment at the mouth, which a node inside the socket roughly halves. Give the diameter, wall, engagement, the loads and the strengths you trust, and it reports the bond area, the governing shear, the pull-out capacity and safety, the bearing stress and safety, the moment the joint takes, and the engagement a pull-out load needs at your safety factor. |
| `inspect.parametric_check` | The modelled-against-drawn test of a design review, done by the machine. Every user parameter (and with `include_model`, every sketch dimension and feature input that is a plain number) is nudged down and up by a percentage on a scratch copy and the model rebuilt. Each direction is classified: the model **follows** (a body's volume, box, centroid or second moments changed), the parameter is **inert** (nothing downstream changed: it drives nothing), a feature **fails**, a body **empties**, the model **detaches** into more shells or separate pieces **merge**. The design itself is never touched. |

## Tolerance chain in the dialog

The Tolerance Stack dialog takes its chain as text, one contributor per line:

```text
Plate 10 ±0.1
Spacer 5 ±0.05
Pocket 15.5 ±0.1 -1
Cap 2 1.9 2.2
```

A name is optional, `±` may be typed `+-` or `+/-`, two numbers after the nominal are its lower
and upper limits, and a trailing `-1` makes the contributor subtract from the gap.

## Parametric check

```json
{"command": "inspect.parametric_check", "params": {"percent": 10, "include_model": true}}
```

answers with every driver's `down` and `up` outcome (`asked`, `verdict`, `note`), the counts of
parameters that `survived`, are `inert` or `broke`, and the advice: an inert parameter should be
deleted or wired into a dimension, and a parameter that breaks the model at ±10 % is a drawn
dimension masquerading as a parameter.

Every number is validated and capped; a nonsense input is refused with the reason rather than
answered. The tables are public standards data (ISO 286-1, ISO 2768-1, ISO 898-1, ISO 261,
ISO 273, ISO 4762, ISO 4032, ISO 7089, DIN 974-1, ISO 15065, ISO 281).
