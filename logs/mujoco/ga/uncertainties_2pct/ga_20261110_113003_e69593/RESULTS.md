# GA with ±2% parametric uncertainties — final result

Search `ga_20261110_113003_e69593`, completed on 10 November 2026.

- 12 individuals, 10 generations, 84 training evaluations, 56 distinct genomes.
- 5 resampled scenarios per generation, 11 common selection scenarios, 11 independent validation scenarios.
- ±2% on armature, damping, joint friction and motor gain, separately for SX/DX.
- IMU and encoder noise; PWM cap 50%, duration 10 s, initial pitch 3°, one-sample delay.
- Selection cost: 13.698971 → 0.781482.
- Validation cost: 13.550557 → 0.779242 (94.25% reduction).
- 0/11 scenarios meet all settling criteria for either controller.

The aggregate cost is mean + 0.5 × maximum. Ranking prioritizes scenarios
that fail the criteria, then missing time, and finally cost. The cost
improvement does not certify full stabilization or real-world performance.
No controller on the board was replaced.

Logs and configuration are in this directory. SVG/PDF/PNG figures are in
`logs/mujoco/ga/plot/`, with prefix `ga_20261110_113003_e69593_`:
nominal comparison, all validation scenarios and selection comparison.
No cumulative cross-generation fitness curve was plotted because each
generation uses different scenarios.
