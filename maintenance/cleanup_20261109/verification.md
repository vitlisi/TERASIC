# Cleanup verification — 9 November 2026

- All 12 protected model/configuration/code inputs retain their original SHA256.
- All 38 relocated items verified against the preservation manifest. 37 are exact copies; one documentation summary was explicitly rewritten, retaining its original hash in the manifest. Numerical evidence was not rewritten.
- Compensation data regenerated from the relocated candidate/source profile are byte-identical to the active C++ header (SHA256 afc30b129e705246a46f053f15987c6d3925c8765d1c57ff5b5b6c0d76a9d090).
- Build script compiled all retained standard and diagnostic programs on the PC; these are not newly qualified ARM binaries.
- 21 C++ offline/RAM tests passed, including bounded Stribeck compensation and hardware-stop logic.
- 57 selected Python integrity/parity/interface tests passed. Necessary missing script dependencies discovered during cleanup were restored before this final check.
- LQR/H-infinity one-second CLI smoke simulations succeeded with the current Stribeck model. GA stand (1 s) and path (33 s) succeeded through the preserved delayed C++ bridge; summaries record the variant and source hashes. These temporary smoke outputs are not final benchmark data.
- Supplementary inverse dynamics reproduced successfully using the relocated evidence and the unchanged mathematics.
- Temporary source staging and Python/tool caches were permanently removed after checks.
- No board connection, motor activation, parameter fitting, controller retuning or final acquisition occurred.

## Visible limitations; full suite is not green

The initial broad Python run was interrupted and is not reported as a complete pass.
Focused runs confirmed these existing-model/default-physics expectations fail:

- `test_ga_robust.RobustTests.test_two_short_rollouts_record_effective_uncertainties_and_noise`: damping metadata expectation differs from current Stribeck branch parameters.
- `test_hinf_controller.HinfTests.test_body_force_rejection_and_cleanup`
- `test_hinf_controller.HinfTests.test_lqr_comparison_same_cap_and_delay`
- `test_hinf_controller.HinfTests.test_sensor_hold_release_perturbation_and_telemetry`
- `test_path_reference.PathTests.test_noisy_sensors_battery_and_endpoint_hold`
- `test_sensor_models.SensorTests.test_full_pipeline_keeps_balance_and_logs_all_axes`

The last five fail their settling/completion expectations with the current default physics. They remain in the repository. No threshold or physics parameter was modified to hide these results. The 57-test integrity check excludes the two performance cases from its selected modules; the H-infinity performance/GA robustness modules are not part of that selected check. These checks establish preservation and executable interfaces, not loaded fidelity or readiness of every controller for hardware.

Final diagram list, matched configurations and compensation setting remain to be specified. Inverse dynamics is supplementary future work only.

## Portable command additions after cleanup

Named simulation runs were smoke-checked with LQR, baseline H-infinity, delayed GA and Python PD outer feedback. Each produced data and automatically exported figures. The existing real supplementary CSV was used only to check the standalone plotter in /tmp, not relabeled as final data. Seven launcher/logging checks passed, including all six controller launchers with custom output paths and missing-battery refusal. No physical controller was started. The current download contains tools and source inputs; final real comparisons remain unacquired.

## Historical digital-twin recovery requested after cleanup

Section 01 now reuses historical board copies; no new motor acquisition or fitting was performed. Identification-grid records were separated from pilot/reference diagnostics through original acquisition names and the cleanup file inventory. The complete 328-point 0.5% grid, four uncompensated pulse runs, 80 moving-state static-LUT points and one complete bounded-compensator run are retained with raw-file SHA256 and source-path provenance. Four classifier fixtures and eight freewheel/GA-core checks passed. The native compensated replay accepted all 2,601 samples and final-OFF evidence. Eleven English SVGs were regenerated and visually checked, including complete traces and additional zooms. The frozen model/compensation/controller hashes remain unchanged. A second full SVG regeneration reused per-point cache only when physical-source and input-file hashes matched. Historical multi-file sizes establish recovery association; they are not asserted as old cryptographic checksums.
