"""Versioned, decomposable stabilization rewards; no torch/Gym dependency."""
import math
import numpy as np


def validate_reward(config):
    keys = {'pitch','pitch_rate','position','speed','yaw','yaw_rate','pwm'}
    if set(config['weights']) != keys or any(not math.isfinite(v) or v<0 for v in config['weights'].values()):
        raise ValueError('Pesi reward non validi')
    if not math.isclose(sum(config['weights'].values()),1.,abs_tol=1e-10):
        raise ValueError('La somma dei pesi reward deve essere uno')
    if any(not math.isfinite(v) or v<=0 for v in config['scales'].values()):
        raise ValueError('Scale reward positive e finite richieste')
    if config.get('shape','bounded') not in ('bounded','pseudo_huber'):
        raise ValueError('Forma reward non valida')
    scale=config.get('output_scale',1.)
    if not math.isfinite(scale) or scale<=0:
        raise ValueError('Scala reward positiva e finita richiesta')
    bonus=config.get('goal_bonus',0.)
    if not math.isfinite(bonus) or bonus<0:
        raise ValueError('Bonus obiettivo non valido')
    if not math.isfinite(config['failure_reward']) or config['failure_reward']>=0:
        raise ValueError('Penalita fallimento negativa richiesta')


def reward_terms(state, pwm, max_pwm, config, failed=False):
    """state is physical truth, used only for reward/evaluation, never observations."""
    s = config['scales']
    normalized = dict(pitch=state['pitch_error_deg']/s['pitch_deg'],
        pitch_rate=state['pitch_rate_deg_s']/s['pitch_rate_deg_s'],
        position=state['position_m']/s['position_m'],speed=state['speed_m_s']/s['speed_m_s'],
        yaw=state['yaw_deg']/s['yaw_deg'],yaw_rate=state['yaw_rate_deg_s']/s['yaw_rate_deg_s'])
    if config.get('shape','bounded')=='pseudo_huber':
        # Quadratic near zero, linear far away: drift keeps increasing cost.
        penalties={k:config['weights'][k]*(math.hypot(1.,v)-1.) for k,v in normalized.items()}
    else:
        penalties = {k:config['weights'][k]*(1-1/(1+min(abs(v),1e100)**2)) for k,v in normalized.items()}
    penalties['pwm'] = config['weights']['pwm']*float(np.mean((np.clip(pwm,-max_pwm,max_pwm)/max_pwm)**2))
    in_goal=(abs(state['pitch_error_deg'])<.3 and abs(state['position_m'])<.01
             and abs(state['yaw_deg'])<2. and abs(state['speed_m_s'])<.02)
    bonus=config.get('goal_bonus',0.) if in_goal and not failed else 0.
    reward = config['failure_reward'] if failed else 1-sum(penalties.values())+bonus
    if config.get('goal_bonus',0.):penalties['goal_bonus']=-bonus
    scale=config.get('output_scale',1.)
    return float(scale*reward),{k:float(scale*v) for k,v in penalties.items()}
