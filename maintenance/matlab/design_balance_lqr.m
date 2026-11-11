function result = design_balance_lqr(A_phys, B_phys_duty, limits, Ts)
% Only for an ALREADY VERIFIED continuous physical realization:
% z=[theta_relative_to_chassis; psi_pitch; theta_dot; psi_dot], rad and rad/s.
% B_phys_duty is 4x1 for SAME normalized signed duty applied to both wheels.
% No hidden factor 0.441: conversion must already be in B.
% limits=[theta rad; psi rad; theta_dot rad/s; psi_dot rad/s; duty fraction].
% Inf for theta means no position regulation: design on the 3-state subsystem.
% Validity of the mechanical model, equilibrium and signs is caller's responsibility.
if nargin<4, Ts=0.01; end
assert(isequal(size(A_phys),[4 4]) && isequal(size(B_phys_duty),[4 1]));
assert(all(isfinite(A_phys(:))) && all(isfinite(B_phys_duty(:))));
assert(numel(limits)==5 && all(limits>0));
assert(all(isfinite(limits(2:5))) && limits(5)<=1 && Ts>0 && isfinite(Ts));
% The first coordinate is a free encoder origin, not a force/position dependency.
assert(norm(A_phys(:,1),inf)<1e-9,'Wheel angle is not cyclic: review state mapping.');
assert(norm(A_phys(1,:)-[0 0 1 0],inf)<1e-9,'Wrong state order or units.');
assert(norm(A_phys(2,:)-[0 0 0 1],inf)<1e-9,'Wrong pitch order or units.');
active=1:4;
if isinf(limits(1)), active=2:4; end
continuous=ss(A_phys(active,active),B_phys_duty(active),eye(numel(active)),zeros(numel(active),1));
discrete=c2d(continuous,Ts,'zoh');
Q=diag(1./limits(active).^2);
R=1/limits(5)^2;
[K,S,poles]=dlqr(discrete.A,discrete.B,Q,R);
assert(all(abs(poles)<1),'Closed-loop poles outside unit circle.');
result.K_duty=zeros(1,4);
result.K_duty(active)=K;
result.K_pwm_percent=100*result.K_duty;
result.Q=Q; result.R=R; result.S=S;
result.Ad=discrete.A; result.Bd=discrete.B;
result.closed_loop_poles=poles;
result.Ts=Ts;
result.state_order={'theta','psi','theta_dot','psi_dot'};
result.full_state_order={'theta','psi','rho','theta_dot','psi_dot','rho_dot'};
result.reduced_state_indices=[1 2 4 5]; % MATLAB 1-based selection from x
result.K_full_pwm_percent=zeros(1,6);
result.K_full_pwm_percent(result.reduced_state_indices)=result.K_pwm_percent;
result.active_states=active;
result.note=['Provisional model design. Include encoder velocity filter, ' ...
    'IMU estimator, saturation/deadband and uncertain parameters in validation.'];
disp(result);
end
