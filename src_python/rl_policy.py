"""Small asymmetric PPO actor/critic and portable deterministic NumPy actor."""
import copy
import json
from pathlib import Path

import numpy as np


def actor_action(actor, observation):
    observation = np.asarray(observation,dtype=float)
    if observation.shape not in ((6,),(7,)) or not np.isfinite(observation).all():
        raise ValueError('Osservazione normalizzata a 6/7 componenti richiesta')
    x = observation[:6]
    for index,layer in enumerate(actor['layers']):
        x = np.asarray(layer['weight'])@x+np.asarray(layer['bias'])
        if index<2:
            x = np.tanh(x)
    return np.clip(x,-1.,1.)


def load_actor(path):
    actor = json.loads(Path(path).read_text())
    if (actor['architecture'] != [6,8,8,2] or actor['activation']!='tanh_hidden_linear_output'
            or actor['observation_order'] != ['theta','psi','rho','theta_dot','psi_dot','rho_dot']):
        raise ValueError('Convenzioni actor non compatibili')
    for layer,shape in zip(actor['layers'],[(8,6),(8,8),(2,8)]):
        w,b = np.asarray(layer['weight']),np.asarray(layer['bias'])
        if w.shape!=shape or b.shape!=(shape[0],) or not np.isfinite(w).all() or not np.isfinite(b).all():
            raise ValueError('Pesi actor non validi')
    if len(actor['layers'])!=3:
        raise ValueError('Richiesti tre strati actor')
    return actor


def make_policy_class():
    # Imported lazily: exported actor evaluation itself needs only NumPy.
    import torch
    from torch import nn
    from stable_baselines3.common.policies import ActorCriticPolicy

    class SplitNetwork(nn.Module):
        def __init__(self):
            super().__init__()
            self.latent_dim_pi = self.latent_dim_vf = 8
            self.policy_net = nn.Sequential(nn.Linear(6,8),nn.Tanh(),nn.Linear(8,8),nn.Tanh())
            self.value_net = nn.Sequential(nn.Linear(7,8),nn.Tanh(),nn.Linear(8,8),nn.Tanh())

        def forward_actor(self, features):
            return self.policy_net(features[...,:6])

        def forward_critic(self, features):
            return self.value_net(features)

        def forward(self, features):
            return self.forward_actor(features),self.forward_critic(features)

    class SmallPolicy(ActorCriticPolicy):
        def __init__(self,*args,**kwargs):
            kwargs.setdefault('log_std_init',-2.302585092994046)
            super().__init__(*args,**kwargs)

        def _build_mlp_extractor(self):
            self.mlp_extractor = SplitNetwork().to(self.device)

    return SmallPolicy


def export_actor(model, config):
    import torch
    layers = [model.policy.mlp_extractor.policy_net[0],model.policy.mlp_extractor.policy_net[2],model.policy.action_net]
    actor = dict(schema_version=1,architecture=[6,8,8,2],activation='tanh_hidden_linear_output',
        observation_order=['theta','psi','rho','theta_dot','psi_dot','rho_dot'],
        observation_scales=config['observation_scales'],
        output_order=['normalized_pwm_sx','normalized_pwm_dx'],max_pwm=config['max_pwm'],
        feedback_delay_samples=config['feedback_delay_samples'],dt_s=config['dt_s'],
        config=copy.deepcopy(config),
        layers=[dict(weight=l.weight.detach().cpu().numpy().tolist(),bias=l.bias.detach().cpu().numpy().tolist()) for l in layers])
    rng = np.random.default_rng(9876)
    tests = np.column_stack([rng.normal(size=(256,6)),rng.uniform(0,1,256)]).astype(np.float32)
    predicted,_ = model.predict(tests,deterministic=True)
    exported = np.array([actor_action(actor,x) for x in tests])
    error = float(np.max(np.abs(predicted-exported)))
    if error>2e-5:
        raise ValueError('Export actor non equivalente a PyTorch: '+str(error))
    actor['export_max_action_error'] = error
    actor['export_test_observations'] = len(tests)
    return actor
