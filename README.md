# IEEE Stock Car Race

SoutheastCon 2027 Hardware Competition | Game Manual v2.0 (6/15/2026)

---

## Quick Summary

Autonomous robot races 3 laps on a 4x8 ft oval track, makes one pit stop to swap tires, and raises a school flag on the final lap. 3 minute time limit. No external control during the match.

---

## Docs

| Doc                              | What it covers                                                           |
| -------------------------------- | ------------------------------------------------------------------------ |
| [field.md](field.md)             | Field dimensions, zones, paint colors, obstacle placement, tolerances    |
| [scoring.md](scoring.md)         | Full points table, speed bonus math, max score breakdown                 |
| [materials.md](materials.md)     | Bill of materials, prices, paint codes, center line method               |
| [robot-specs.md](robot-specs.md) | Size constraints, autonomy rules, tire carry, flag mechanism, inspection |
| [pit-stop.md](pit-stop.md)       | Die roll table, pit sequence, tire exchange rules, 75% in rule           |
| [pre-match.md](pre-match.md)     | Field and technician setup checklists, randomization rules               |
| [match-play.md](match-play.md)   | Start trigger, race flow, flag raising, timer rules                      |
| [post-match.md](post-match.md)   | Shutdown procedure, auto-shutoff, retrieval rules                        |
| [apriltags.md](apriltags.md)     | Tag specs, ID to pit spot mapping, detection tips                        |
| [penalties.md](penalties.md)     | Yellow and Red card triggers, scoring impacts                            |
| [faq.md](faq.md)                 | Common questions, update with Q&A responses as they come in              |

---

## Scoring

| Achievement                                   | Points                 |
| --------------------------------------------- | ---------------------- |
| Robot enters the course                       | 25                     |
| Finish in 3 minutes or less                   | 100                    |
| Speed bonus, 1 pt per full 10 sec under 3 min | Various                |
| Start from green flag                         | 10                     |
| Making a pit stop                             | 15 (10 if under 2 sec) |
| Stopping in correct pit spot                  | 15                     |
| Dropping tire in pit                          | 25                     |
| Picking up fresh tire                         | 40                     |
| Ending race in assigned pit spot              | 15                     |
| Clean lap, no wall contact (per lap)          | 10                     |
| Clean lap, no obstacle contact (per lap)      | 15                     |
| Raising school flag on final lap              | 25                     |
| Entering publicity contest                    | 10                     |

---

## Key Rules

- Robot must be fully autonomous, no external signals during the match
- Starting before the green flag eliminates the 10 point bonus
- Pit stop must happen after lap 1 or lap 2, not after lap 3
- More than 75% of the robot must be inside the pit spot to count
- Picked up tire must be different from the one dropped
- Field and game elements may vary by ± 1 in. per match, design accordingly
- Do not interact with obstacles or the Apriltag after they are placed

---

_Official Q&A responses from the GDC take precedence over the game manual. Log clarifications in [faq.md](faq.md)._
