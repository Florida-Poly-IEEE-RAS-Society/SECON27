# Apriltags

## What They Are
Apriltags are visual fiducial markers for robotics. Your robot's camera reads the tag ID to determine which pit spot it is assigned to.

Reference: https://april.eecs.umich.edu/software/apriltag

---

## Tag Specs

| Property | Value |
|---|---|
| Family | 41h12 |
| Size | 68mm |
| IDs used | 1, 2, 3 |

---

## ID to Pit Spot

| Tag ID | Assigned Pit Spot |
|---|---|
| 1 | Spot 1 |
| 2 | Spot 2 |
| 3 | Spot 3 |

---

## Placement
- Mounted on the wall near the start area
- One tag selected and mounted before each match
- Do not interact with the tag after it is placed — doing so voids Apriltag related scoring

---

## Physical Handling
- Cut tags along the dashed lines
- Laminate before use

---

## Detection Tips
- Read the tag early, ideally as the robot first approaches the pit area
- Tag is at a fixed location so you can aim your camera accordingly
- Test detection under varying light conditions since the competition venue may differ from your lab
