"""ASCII telemetry in three viewer corners; no controller or physics inputs."""
import math
import mujoco


def _value(value, precision=2, signed=True):
    if value is None or not math.isfinite(value):
        return 'n/d'
    return format(value, ('+' if signed else '')+'.'+str(precision)+'f')


def _degrees(value):
    return None if value is None else math.degrees(value)


def _block(position, rows):
    return (mujoco.mjtFontScale.mjFONTSCALE_150, position,
            '\n'.join(label for label, _ in rows),
            '\n'.join(value for _, value in rows))


def build_hud(phase, mode, info, voltage, torques, disturbance, battery=None,
              battery_load=None, stopped_reason=None, force_y_n=0.,
              actual_start_pitch_deg=None):
    """Only use values already logged at the control sample, with explicit units."""
    motion = mode in ('lqr-motion', 'lqr-path', 'hinf')
    stopped = stopped_reason is not None
    phase_names = dict(settling='Assestamento', calibration='Calibrazione',
                       reposition='Posizionamento', free='Controllo attivo')
    state = [('ASSETTO', 'FERMO' if stopped else phase_names.get(phase, phase)),
             ('Theta [deg]', _value(_degrees(info.get('x_theta_rad')))),
             ('Pitch [deg]', _value(info.get('pitch_est_deg'))),
             ('Yaw [deg]', _value(_degrees(info.get('x_rho_rad'))))]
    state += [('Partenza effettiva [deg]', _value(actual_start_pitch_deg)),
              ('Ultrasuoni [m]', _value(info.get('ultrasonic_distance_m'), 3, False))]
    if info.get('ultrasonic_stop_active') and not stopped:
        state[0] = ('ASSETTO', 'Arresto ultrasuoni')
    motion_rows = [('POSA / ERRORI', 'Al termine' if stopped else '')]
    if motion:
        motion_rows += [
            ('s / rif. [m]', _value(info.get('position_encoder_m'), 3, False)+'/'+_value(info.get('distance_ref_m'), 3, False)),
            ('v / rif. [m/s]', _value(info.get('velocity_encoder_m_s'), 3, False)+'/'+_value(info.get('velocity_ref_m_s'), 3, False)),
            ('X / Y [m]', _value(info.get('pose_est_x_m'), 3, False)+'/'+_value(info.get('pose_est_y_m'), 3, False)),
            ('Err s [mm]', _value(None if info.get('distance_error_m') is None else 1000*info['distance_error_m'])),
            ('Err yaw [deg]', _value(_degrees(info.get('yaw_error_rad')))),
        ]
        if mode == 'lqr-path':
            # Replace the arc-distance error with spatial path error, without
            # increasing the block size on narrow windows.
            error = info.get('path_pose_error_m')
            motion_rows[4] = ('Err posa [mm]', _value(None if error is None else 1000*error))
    else:
        motion_rows += [('Distanza [m]', _value(info.get('position_encoder_m'), 3)),
                       ('Velocita [m/s]', _value(info.get('velocity_encoder_m_s'), 3))]
    pitch, target = info.get('pitch_est_deg'), info.get('pitch_target_deg')
    motion_rows += [('Pitch rif. [deg]', _value(target)),
                    ('Err pitch [deg]', _value(None if pitch is None or target is None else pitch-target))]
    if stopped:
        # Viewer fonts are ASCII; preserve accented stop reasons legibly.
        import unicodedata
        reason = unicodedata.normalize('NFKD', stopped_reason).encode('ascii', 'ignore').decode('ascii')
        state[0] = ('ASSETTO', 'FERMO - '+reason)

    left = info.get('pwm_sx_applied_percent', info.get('pwm_applied_percent', 0.))
    right = info.get('pwm_dx_applied_percent', info.get('pwm_applied_percent', 0.))
    if stopped:
        left = right = 0.
        torques, disturbance = (0., 0.), 0.
        force_y_n = 0.
    power = [('BATT / MOTORI', 'SX/DX'),
             ('PWM [%]', _value(left, 1, False)+'/'+_value(right, 1, False)),
             ('Coppia [Nm]', _value(torques[0], 4, False)+'/'+_value(torques[1], 4, False)),
             ('Batteria [V]', _value(voltage, 2, False))]
    if battery is not None:
        power += [('SOC [%]', _value(100*battery.soc, 3, False)),
                  ('Usati [mAh]', _value(1000*battery.used_ah.sum(), 2, False)),
                  ('Restanti [mAh]', _value(1000*battery.capacity_ah*battery.soc, 1, False))]
        # The physical model remains loaded by electronics even with PWM zero.
        current = None if battery_load is None else battery_load.get('battery_current_A')
        power += [('I batteria [A]', _value(current, 3, False))]
    power += [('Coppia dist. [Nm]', _value(disturbance, 3)),
              ('Forza Y mondo [N]', _value(force_y_n, 3))]
    return [_block(mujoco.mjtGridPos.mjGRID_TOPLEFT, state),
            _block(mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, motion_rows),
            _block(mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT, power)]
