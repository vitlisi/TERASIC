# Controller logs

New public launch commands write files directly into:

```text
logs/
  real/{pd,pd-outer,lqr,hinf,ga}/
  mujoco/{pd,pd-outer,lqr,hinf,ga}/
```

Braces above list the five controller directories under each source.
Run timestamp/unique ID, stand/path and real shadow/run mode are in filenames.
There is no directory per ordinary run. Keep files with the same prefix together:
CSV is the trace, JSON is simulation metadata, `.status.log` is real termination
and diagnostics, `.config.txt` is the real controller configuration snapshot.
PD stores `.args.txt` because its configuration is positional command arguments.
Simulation `experiment.json` manifests enumerate the files for each run.
Plots go to `plot/` inside the controller directory, with the same prefix.
For real CSVs, `scripts/plot_real.py CSV` defaults to the adjacent `plot/` folder.
The public entry point is `scripts/run_experiment.py`; lower-level development
scripts may retain their own layout. See the root README for launch commands.

Existing real and simulated acquisitions have been migrated into the same source/controller layout.
Historical file groups have stable test prefixes. Raw log bytes are preserved;
old paths inside raw metadata describe their original location. Consult
`maintenance/log_layout_migration/manifest.json` for old/new paths.
Digital-twin calibration and GA search checkpoints retain necessary internal
structure under their respective source directories. Old campaign planning
files are retained under `maintenance/log_layout_migration/previous_layout/`.
The ±2% GA has completed. Its final logs are under `mujoco/ga/uncertainties_2pct/`
and its figures under `mujoco/ga/plot/`. The temporary old-path compatibility
link has been removed. A checksummed backup is retained in `archives/ga/`.
