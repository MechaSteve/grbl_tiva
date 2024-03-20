# grbl_tiva
grbl ported to TI Stellaris LM4F / TM4C processors

## Purpose
This is posted for two reasons:
1. Public repositories are free
2. Anyone else wanting to port to the same or similar platform may use this as a starting point.

## Development Plan
- [x] Branch grbl and import into CCS v6
- [x] Fix syntax, add missing includes, refactor with StellarisWare functions. Get programs to compile without error [DONE]
- [x] Test and bug fix to get at least a single axis working [DONE]
- [x] Test/Fix Multi-Axis motion
- [x] Limit Switches
- [x] Spindle control
- [x] Actually running G-Code
- [ ] Probing
- [ ] Replace AMASS & Breshem with timer outputs
  * Each axis gets a 32 bit timer
  * Each axis has an interrupt that updates axis position
  * Axis have a queue of periods and counts
