# Supplied Simscape/Simulink source set

Selected unchanged scripts/models/STL from the supplied26October archive. `provenance.json` retains identities; caches/backups were not imported. Keep this directory with the project when reproducing MATLAB work: it is local material excluded by the historical Git ignore rules.

`init_terasic_2PWM.m` defines parameters, analytic six-state model, LQR and plots; `Terasic_Simscape_LQR_2PWM.slx` is the nonlinear two-wheel-input model; `Terasic_Simscape_linmod_2PWM.slx` is the linearization variant. Earlier models in the parent directory remain historical sources. See [MATLAB workflow](../../../scripts/hinf/README.md); MATLAB, Simulink, Simscape Multibody and required control toolboxes are not installed/validated by the Python documentation pass.

## Retained field/file identifiers

`provenance.json`.

The original README is preserved at its repository-relative path inside `maintenance/review_20261107/original_readmes.tar.gz`. Numerical/source evidence is not rewritten by this English guide.
