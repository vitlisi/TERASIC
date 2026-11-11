# Stopped exploratory study: limitations

The unloaded motor model reproduced much of the pulse-response data, but that
does not fix the absolute torque/inertia ambiguity or validate loaded motion.
The loaded pitch-rate oscillation remained different from the real robot.
Inverse body dynamics was evaluated conditionally using CAD parameters, IMU and
encoders; no independent torque/current sensor or ground-truth slip measurement
was available. Differentiation, timing and contact uncertainty affect the result.

Exploratory common torque scaling, effective timing and rolling-contact candidates
did not consistently match all independent metrics. None was adopted. Compact
metrics are retained in `exploratory_metrics.csv` as context, not as validation.
The user stopped the study; any further identification is future work requiring
an independent experimental protocol. Final benchmark results must come from
new matched real/MuJoCo trials, not these discarded candidate comparisons.
