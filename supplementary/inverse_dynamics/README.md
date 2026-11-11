# Supplementary inverse dynamics — study stopped

Retained only as supplementary evidence and possible future improvements for
the final report. No inferred torque scale, input delay, rolling-contact change
or controller correction from this study is installed in the frozen model.
It does not establish loaded-model validation or absolute measured motor torque.

`data/earlier/` and `data/later/` contain the two minimal real compensated LQR
records, their status/register evidence and run context. `data/synthetic_control.csv`
is a model-generated control case, not a real measurement. `results/` preserves
the original final inverse-dynamics analysis, with its original provenance fields.
The cleanup [preservation manifest](../../maintenance/cleanup_20261109/preservation_manifest.json)
maps old paths to these retained paths and records exact SHA256 values.
`source_snapshot/` and `original_method.md` document the original analysis;
old path strings inside that historical material are not current entry points.

Reproduce the relocated evidence offline, writing new temporary outputs:

```sh
env/bin/python -B diagnostics/inverse_body_torque.py --output /tmp/robot01_inverse_supplement
```

Interpretation depends on CAD mass/inertia/geometry, sensor alignment, derivatives
and no-slip assumptions. Near zero speed, friction yields an interval rather
than a uniquely identifiable applied torque. Few moving windows and correlated
signals prevent an independent identification of all motor/contact parameters.
See [exploratory limitations](exploratory_limitations.md). Further measurement
and independent validation would be required before adopting a correction.
