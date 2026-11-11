# GA tuning of the six-state LQR

This is offline MuJoCo stabilization with six states and two PWM inputs. The
GA searches design weights; the resulting balancing controller is an ordinary
LQR with fixed K. No PyTorch dependency is needed. Current searches can optimize
eight diagonal weights (six Q and separate positive SX/DX R); `--fixed-r` selects
the older six-gene/fixed-R comparison. No hardware installation is automatic.

## Run and inspect

```sh
env/bin/python scripts/ga/tune_lqr_ga.py --help
env/bin/python scripts/ga/tune_lqr_ga.py --population 8 --generations 4
env/bin/python scripts/ga/tune_lqr_ga.py --population 12 --generations 10 --seed 42 --optimize-r
```

A short run checks the workflow; it does not certify robustness. Use the same
seed, model, scenario set and budget for meaningful comparisons. The explicit
configuration is `config/ga/lqr_stabilization.json`; command-line options override
population, generations, seeds and scenario counts. Every run creates a new
folder under `logs/mujoco/ga/`. `--no-plots` suppresses figures only.

The model profile `config/mujoco_motors_sx_dx_20261030.json` is applied before
linearization and rollout; the nominal XML is not rewritten. Armature/damping/
ON-OFF friction are distinct per wheel. Dry friction is excluded only for the
local differentiable linearization and retained in nonlinear rollouts. The
discrete A/B design is frozen across candidates; profiles/provenance are stored
in `nominal_design.json`, `best.json` and the run snapshot.

## Objective and selection

The pipeline designs a baseline, includes it in the population, synthesizes K
from candidate weights, evaluates common training scenarios, and applies
selection/crossover/mutation. The winner is evaluated on a separately declared
scenario set. Do not present selection-set results as independent validation.
The completed historical27October and updated3November studies use different
models/R choices and are not interchangeable benchmarks.

Examples of changing one setting:

```sh
env/bin/python scripts/ga/tune_lqr_ga.py --population 12 --generations 10 --seed 42 --optimize-r --cost-pitch 8
env/bin/python scripts/ga/tune_lqr_ga.py --population 12 --generations 10 --seed 42 --r-scale 2
```

Fitness weights are stored with each run. Different fitness weights produce
different objectives; compare physical metrics or recompute a common cost.
`--r-scale` scales the nominal R/design search center, not the fitness weights.
Larger R does not guarantee lower nonlinear closed-loop PWM because Q can also
change. The selected configuration is not silently replaced by a new search.

## Final controller reproduction

The transferred fixed profiles are `config/controllers/ga_stand.txt` and
`ga_path.txt`. Use `scripts/simulate_ga_twin.py simulate --case both` for the
real C++ cores including the transferred one-sample feedback delay. This is
separate from rerunning optimization. Old historical result datasets and their
plot catalogues were permanently deleted. A new full 12-individual,
10-generation stand search with current Stribeck physics and a 20% cap completed
on 10 November 2026. See [results and figure commands](../../docs/GA_OPTIMIZATION_20261110.md).
