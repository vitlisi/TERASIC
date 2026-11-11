clc
clear
% =========================================================================
% Parametri Digital Twin - Terasic Self-Balancing Robot
% File di inizializzazione, linearizzazione e progetto LQR (2 PWM)
% =========================================================================

%% 1. Parametri ambientali
g = 9.81; % accelerazione di gravita' [m/s^2]

%% 2. Parametri telaio (Skeleton)
chassis_mass = 0.752; % massa corpo centrale [kg]
chassis_com = [0.00030, 0.00053, 0.03503]; % centro di massa [m]
chassis_I_diag = [0.000895, 0.001450, 0.000970]; % [Ixx Iyy Izz] [kg*m^2]
chassis_I_prod = [0.000009, -0.000004, 0.000021]; % [Ixy Ixz Iyz] [kg*m^2]

%% 3. Geometria ruote e posizione
wheel_mass = 0.049; % massa ruota [kg]
wheel_radius = 0.032; % raggio ruota [m]
wheel_thickness = 0.028; % spessore ruota [m]
wheel_offset_x = 0.082; % distanza ruote dall'origine [m]
wheel_track = 0.1644; % distanza tra i centri delle ruote [m]

%% 4. Parametri motore e giunti
motor_max_torque = 0.441; % coppia massima/stallo [N*m]
joint_damping = 0.0126; % smorzamento viscoso stimato [N*m/(rad/s)]
joint_friction = 0.01; % attrito statico stimato [N*m]
rotor_inertia = 0.005; % inerzia rotore riflessa [kg*m^2]

%% 5. Parametri di contatto
mu_static = 0.6;
mu_kinematic = 0.5;

%% 6. PWM -> coppia
I_stall = 1; % corrente di stallo [A]
V_nom = 12; % tensione nominale [V]
R_int = V_nom/I_stall; % resistenza interna [ohm]
gear = 30; % rapporto di riduzione
k_t = motor_max_torque/(gear*I_stall); % costante di coppia [N*m/A]

%% 7. Parametri temporali
dt = 0.01; % periodo di campionamento del controllore [s]

%% 8. Modello analitico linearizzato a 6 stati (2 PWM)
% Coordinate generalizzate:
%   q = [theta; psi; phi]
%   theta = avanzamento longitudinale / wheel_radius [rad equivalenti]
%   psi   = pitch del telaio, positivo in avanti [rad]
%   phi   = yaw [rad]
%
% Si assume rotolamento senza slittamento e ruote identiche.
% L'angolo encoder di ciascuna ruota e' RELATIVO al telaio. Con la
% convenzione adottata qui: qwheel_rel = theta - psi +/- d*phi.
% Se il verso del Pitch nel modello Simscape e' opposto, cambiare
% sigma_pitch_encoder da -1 a +1 e verificare i segni con una prova breve.
%
% Ipotesi parametriche da verificare:
%   - rotor_inertia e' l'inerzia riflessa DI CIASCUN motore all'asse ruota;
%   - inerzia ruota stimata come disco pieno;
%   - PWM normalizzato [-1,1], coppia lineare attorno a PWM=0;
%   - attrito statico e saturazioni non linearizzati.

sigma_pitch_encoder = -1; % qwheel_rel = theta + sigma_pitch_encoder*psi +/- d*phi
wheel_inertia = 0.5 * wheel_mass * wheel_radius^2; % stima disco pieno [kg*m^2]
rotor_inertia_per_wheel = rotor_inertia; % [kg*m^2], ipotesi: valore per ciascun motore
half_track_over_radius = wheel_track / (2 * wheel_radius);

m = chassis_mass;
mw = wheel_mass;
r = wheel_radius;
b = wheel_track;
l = chassis_com(3); % altezza COM sopra asse ruote [m]
Ip = chassis_I_diag(2); % Iyy pitch [kg*m^2]
Iz = chassis_I_diag(3); % Izz yaw [kg*m^2]
Jw = wheel_inertia;
Jm = rotor_inertia_per_wheel;
c = joint_damping;
d = half_track_over_radius;

% Matrice d'inerzia per q = [theta; psi; phi].
% Include traslazione del telaio/ruote, pitch del telaio,
% rotazione delle ruote e inerzia riflessa dei due rotori.
M = [r^2*(m + 2*mw) + 2*Jw + 2*Jm,  m*r*l + 2*sigma_pitch_encoder*Jm, 0;
     m*r*l + 2*sigma_pitch_encoder*Jm, Ip + m*l^2 + 2*Jm,                  0;
     0,                                  0, Iz + mw*b^2/2 + 2*(Jw+Jm)*d^2];

% Smorzamento viscoso ai due giunti ruota. Le velocita' relative
% sono theta_dot + sigma*psi_dot +/- d*phi_dot.
D_mech = 2*c * [1, sigma_pitch_encoder, 0;
                sigma_pitch_encoder, 1, 0;
                0, 0, d^2];

% Linearizzazione del potenziale gravitazionale attorno al pendolo
% verticale instabile: K_mech(2,2) = -m*g*l.
K_mech = diag([0, -m*g*l, 0]);

% Coppie generalizzate dovute a [tau_R; tau_L].
% Ordine ingressi: PWM_R, PWM_L.
G = [1, 1;
     sigma_pitch_encoder, sigma_pitch_encoder;
     d, -d];

torque_per_pwm = gear * k_t * (V_nom / R_int); % [N*m per PWM unitario]

% Realizzazione di stato x = [theta; psi; phi; theta_dot; psi_dot; phi_dot].
A = [zeros(3), eye(3);
     -(M\K_mech), -(M\D_mech)];
B = [zeros(3,2);
     M\G] * torque_per_pwm;
C = eye(6);          % uscita: tutti gli stati
D = zeros(6,2);      % feedthrough diretto PWM -> stato nullo

% Mappa encoder relativi -> stati cinematici secondo la convenzione sopra:
% theta = mean(theta_R,theta_L) - sigma_pitch_encoder*psi
% phi   = (wheel_radius/wheel_track)*(theta_R-theta_L)

fprintf('\n=== Modello analitico lineare 2 PWM ===\n');
fprintf('Stati: [theta; psi; phi; theta_dot; psi_dot; phi_dot]\n');
fprintf('Ingressi: [PWM_R; PWM_L], coppia/PWM = %.6g N*m\n', torque_per_pwm);
fprintf('Inerzia ruota stimata = %.6g kg*m^2; inerzia rotore per ruota = %.6g kg*m^2\n', Jw, Jm);
disp('M ='); disp(M);
disp('D_mech ='); disp(D_mech);
disp('K_mech ='); disp(K_mech);
disp('G ='); disp(G);
disp('A ='); disp(A);
disp('B ='); disp(B);
disp('C ='); disp(C);
disp('D ='); disp(D);

%% 9. Progettazione della LQR

%%% Controllability Check
rank_ctrb = rank(ctrb(A,B));

if rank_ctrb == size(A,1)
    fprintf('\nCompletely controllable system! LQR is possible to project with all states.\n\n')
else
    fprintf('\nNOT completely controllable system!\n\n')
end

%%% LQR

sys_lqr = ss(A,B,C,D);
sys_dlqr = c2d(sys_lqr,dt,'zoh');

A_d = sys_dlqr.A;
B_d = sys_dlqr.B;
C_d = sys_dlqr.C;

R = 10*eye(2);
N = zeros(6,2);
r_1 = 40;
r_2 = 1.2e-2;
Q = r_1*C_d'*C_d + r_2*diag([10, 15, 10, 7, 4, 10]);
max_pwm = 0.15; % max PWM for saturation

[K_dlqr, P_dlqr, poles_dlqr] = dlqr(A_d,B_d,Q,R,N)

%% 10. Simulation
open('Terasic_Simscape_LQR_2PWM.slx');
sim('Terasic_Simscape_LQR_2PWM.slx');

%% 11. Plots

figure(1)
hold on
plot(out.x.Time(:,1), out.x.Data(1,:),'LineWidth',2)
plot(out.x.Time(:,1), out.x.Data(2,:),'LineWidth',2)
plot(out.x.Time(:,1), out.x.Data(3,:),'LineWidth',2)
plot(out.x.Time(:,1), out.x.Data(4,:),'LineWidth',2)
plot(out.x.Time(:,1), out.x.Data(5,:),'LineWidth',2)
plot(out.x.Time(:,1), out.x.Data(6,:),'LineWidth',2)
title('State evolution with LQR and two PWMs', 'FontSize', 12)
xlabel('$t [s]$', 'Interpreter', 'latex')
ylabel('${x}$', 'Interpreter', 'latex')
legend('${\theta}$','${\psi}$','${\phi}$','${\dot \theta}$','${\dot \psi}$','${\dot \phi}$', 'Interpreter', 'latex','Location','best')
grid on
hold off

figure(2)
hold on
plot(out.u.Time(:,1), out.u.Data(:,1),'LineWidth',2)
plot(out.u.Time(:,1), out.u.Data(:,2),'LineWidth',2)
title('Control law evolution with LQR and two PWMs', 'FontSize', 12)
xlabel('$t [s]$', 'Interpreter', 'latex')
ylabel('${u}$', 'Interpreter', 'latex')
legend('${u}_1$', '${u}_2$', 'Interpreter', 'latex')
grid on
hold off

figure(3)
hold on
plot(out.PWM.Time(:,1), out.PWM.Data(:,1),'LineWidth',2)
plot(out.PWM.Time(:,1), out.PWM.Data(:,2),'LineWidth',2)
title('PWMs duty cycle evolution with LQR and saturation', 'FontSize', 12)
xlabel('$t [s]$', 'Interpreter', 'latex')
ylabel('${PWM}$', 'Interpreter', 'latex')
legend('${PWM}_R$', '${PWM}_L$', 'Interpreter', 'latex')
ylim([-0.25 0.25]) % Imposta l'intervallo dell'asse y
grid on
hold off
