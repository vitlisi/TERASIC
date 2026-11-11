# Robot models and controller inputs

| Path | Contents |
|---|---|
| `verifica.xml` | MuJoCo bodies, masses, inertias, joints, contact, motors and sensors |
| `meshes/` | Chassis/wheel STL visual geometry |
| `lqr/` | Hardware/design/screening configuration inputs |
| `hinf/` | Imported MATLAB dynamic-controller matrices |

Nominal model values: chassis0.75166 kg excluding wheels; each wheel0.049 kg;
total0.84966 kg; radius0.032 m; track0.16444 m; nominal supply12 V; assumed
wheel stall torque0.441 N m. Physics timestep1 ms; control period10 ms.
Chassis mass derives from CAD and includes the battery enclosure; wheel masses
were supplied by the operator. Added Wi-Fi payload is an explicit separate
assumption in diagnostic comparisons, not a change to the nominal CAD file.

Motor loss/torque, sensor and battery parameters contain assumptions. The
unloaded fitted speed response does not measure absolute loaded torque. See
[configuration](../config/README.md) and [experimental status](../README.md).
The simulated HC-SR04 uses a0.02–4 m range and70 ms sampling; its15° cone is a
model assumption. Original manuals are in `config/references/`.
