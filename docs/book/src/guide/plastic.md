# Plastic parts

MODEL → CREATE lists the plastic enclosure features:

- **Boss** (`plastic.boss`): screw bosses with a hole, fillets and optional ribs.
- **Lip / Groove** (`plastic.lip`): the mating lip and groove along a shell's opening.
- **Snap Fit** (`plastic.snap_fit`): a cantilever snap hook and its catch.
- **Rest** (`plastic.rest`): a pad for a part to rest on.
- **Grill** (`plastic.grill`, on 3DEEE › Parts): a vent through a flat wall. Click the wall where
  the vent's centre goes: slots of `slot_width_mm` between ribs of `rib_width_mm` run along
  `direction` inside a rim of `rim_width_mm`, cut through the wall under the face, with optional
  cross spars and a raised rim.
- **Plastic Rules** (`plastic.manage_rules`, `plastic.assign_rule`): wall thickness, draft and
  clearance per material, used as the features' defaults.

Plastic Rules are in MANAGE → RULES. Rib and web in MODEL → CREATE complete the set. These features are checked against
hand-computed geometry.
