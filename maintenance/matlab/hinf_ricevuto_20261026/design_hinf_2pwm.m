function result = design_hinf_2pwm(output_dir)
% Prima sintesi nominale discreta, sei misure e due PWM normalizzati.
% Eseguire in MATLAB con Control System Toolbox e Robust Control Toolbox.
if nargin < 1
    output_dir = fullfile(fileparts(mfilename('fullpath')), 'generated');
end
assert(exist('hinfsyn','file') ~= 0, 'Serve Robust Control Toolbox.');
p = terasic_hinf_parameters();

% Scelte iniziali di progetto, NON tolleranze gia' validate sul robot.
state_scale = [0.02/p.wheel_radius; deg2rad(3); deg2rad(5); ...
               0.5; deg2rad(30); deg2rad(30)];
noise_scale = [0.002/p.wheel_radius; deg2rad(0.2); deg2rad(0.5); ...
               0.05; deg2rad(2); deg2rad(2)];
disturbance_duty = 0.03; % due disturbi additivi alle PWM ruota
Wx = diag(1./state_scale);
Wu = eye(2)/p.max_pwm;
V = diag(noise_scale);

% xa=[x(k); x(k-1)]: stesso ritardo esterno del modello SLX.
% w=[disturbo_PWM_R; disturbo_PWM_L; rumore_sui_6_stati].
Aa = [p.A_d, zeros(6); eye(6), zeros(6)];
Bu = [p.B_d; zeros(6,2)];
Bw = [p.B_d*disturbance_duty, zeros(6); zeros(6,8)];
Cy = [zeros(6), eye(6)];
Cz = [Wx, zeros(6); zeros(2,12)];
Dzw = zeros(8,8);
Dzu = [zeros(6,2); Wu];
Dyw = [zeros(6,2), V];
% hinfsyn collega gli ULTIMI 2 ingressi e le ULTIME 6 uscite.
P = ss(Aa, [Bw,Bu], [Cz;Cy], ...
       [Dzw,Dzu; Dyw,zeros(6,2)], p.dt);
[K,CL,gamma] = hinfsyn(P,6,2);
assert(~isempty(K) && isfinite(gamma), 'Sintesi Hinf non riuscita.');
assert(isstable(CL), 'Anello Hinf nominale non stabile.');
[Ak,Bk,Ck,Dk] = ssdata(K);
% Feedback POSITIVO lft: il segno stabilizzante e' gia' nelle matrici K.
Acl = [Aa+Bu*Dk*Cy, Bu*Ck; Bk*Cy, Ak];
rho = max(abs(eig(Acl)));
assert(rho < 1, 'Modo interno instabile: verificare realizzazione/segni.');
assert(abs(K.Ts-p.dt) < 1e-12, 'Periodo controllore inatteso.');

result = struct('schema_version',1,'controller','hinf_2pwm_nominal', ...
    'Ak',Ak,'Bk',Bk,'Ck',Ck,'Dk',Dk,'dt_s',p.dt, ...
    'controller_initial_state',zeros(size(Ak,1),1), ...
    'measurement_delay_samples',1,'measurement_delay_initial_state',zeros(6,1), ...
    'measurement_order',{{'theta','psi','phi','theta_dot','psi_dot','phi_dot'}}, ...
    'measurement_units',{{'rad','rad','rad','rad/s','rad/s','rad/s'}}, ...
    'input_convention','estimated_state_minus_reference_at_equilibrium', ...
    'output_order',{{'PWM_R','PWM_L'}},'output_unit','normalized_duty', ...
    'output_limit',p.max_pwm,'wheel_track_m',p.wheel_track, ...
    'wheel_radius_m',p.wheel_radius,'gamma',gamma, ...
    'closed_loop_spectral_radius',rho, ...
    'state_scale',state_scale,'measurement_noise_scale',noise_scale, ...
    'disturbance_duty_scale',disturbance_duty, ...
    'plant_source','analytic_2PWM_from_updated_archive_20261026', ...
    'validation_status','nominal_linear_only_not_validated_on_Simscape_or_robot');
% LQR originale con lo STESSO ritardo, utile per confrontare la baseline.
lqr_delay_poles = eig(Aa-Bu*p.K_dlqr*Cy);
result.lqr_with_delay_spectral_radius = max(abs(lqr_delay_poles));
if ~exist(output_dir,'dir'), mkdir(output_dir); end
save(fullfile(output_dir,'hinf_2pwm.mat'), ...
    'K','CL','P','p','result','Ak','Bk','Ck','Dk','Acl','lqr_delay_poles');
fid = fopen(fullfile(output_dir,'hinf_2pwm.json'),'w');
assert(fid ~= -1, 'Impossibile aprire JSON di esportazione.');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(result));
fprintf('Hinf: gamma=%.6g, rho=%.6g, ordine=%d, dt=%.3g s\n', ...
    gamma,rho,size(Ak,1),p.dt);
fprintf('Baseline LQR con ritardo: rho=%.6g\n',result.lqr_with_delay_spectral_radius);
fprintf('Esportazione: %s\n',output_dir);
end
