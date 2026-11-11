clc
clear
% =========================================================================
% Parametri Digital Twin - Terasic Self-Balancing Robot
% File di inizializzazione, linearizzazione e progetto LQR (una PWM)
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

%% 8. Linearizzazione del modello a una PWM
model = 'Terasic_Simscape_linmod_1PWM';
load_system(model);

% Specifica dell'operating point
opspec = operspec(model);
for i = 1:length(opspec.States)
    blkName = opspec.States(i).Block;
    if contains(blkName, 'Unit Delay') || contains(blkName, 'State_computation')
        opspec.States(i).Known = 1;
        opspec.States(i).SteadyState = 0;
    end
end

opt = findopOptions('DisplayReport', 'off');
op = findop(model, opspec, opt);
io(1) = linio([model '/Constant'], 1, 'input');
io(2) = linio([model '/State_computation'], 1, 'output');

% Mostra il valore nominale dell'ingresso al punto di equilibrio, quando
% Simulink lo registra come InputPoint (tipicamente per Inport di modello).
fprintf('\n=== Verifica ingresso al punto di equilibrio ===\n');
fprintf('Tempo del punto operativo: %.9g s\n', op.Time);
if isempty(op.Inputs)
    fprintf(['op.Inputs e'' vuoto: il modello non espone InputPoint nel ', ...
             'punto operativo. Questo NON dimostra da solo che u_eq sia zero.\n']);
else
    for k = 1:numel(op.Inputs)
        fprintf('InputPoint %d | blocco: %s | valore u: ', k, op.Inputs(k).Block);
        disp(op.Inputs(k).u);
    end
end

% Elenca i blocchi Step interni e ne calcola il valore nominale all'istante
% op.Time usando i parametri standard Time, Before e After del blocco Step.
% E'' un candidato a u_eq solo se il relativo segnale raggiunge direttamente
% il punto di linearizzazione, senza somma, gain o altra conversione.
stepBlocks = find_system(model, 'BlockType', 'Step');
if isempty(stepBlocks)
    fprintf('Nessun blocco Step interno trovato.\n');
else
    fprintf(['Valori dei blocchi Step all''istante operativo ', ...
             '(candidati, da verificare sul percorso del segnale):\n']);
    for k = 1:numel(stepBlocks)
        stepTime = str2double(get_param(stepBlocks{k}, 'Time'));
        beforeValue = str2double(get_param(stepBlocks{k}, 'Before'));
        afterValue = str2double(get_param(stepBlocks{k}, 'After'));
        if op.Time < stepTime
            stepValueAtOp = beforeValue;
        else
            stepValueAtOp = afterValue;
        end
        fprintf('  %s | Step nominale = %.9g\n', ...
                stepBlocks{k}, stepValueAtOp);
    end
end

% Visualizza i punti di linearizzazione per collegare l'ingresso perturbato
% al segnale nominale corretto nel diagramma.
disp('Punti di linearizzazione getlinio(model):');
disp(io);

[sys_raw, linReport] = linearize(model, io, op);
[A_raw, B_raw, C_raw, D_raw] = ssdata(sys_raw);
stateNames = string(sys_raw.StateName);

fprintf('Ordine della realizzazione completa: %d stati\n', order(sys_raw));
disp('Nomi degli stati della realizzazione completa:');
disp(sys_raw.StateName);
disp('Stati riportati dalla linearizzazione:');
disp(linReport.States);

% Mappatura inferita dalla topologia del modello:
% Revolute_Joint  = inclinazione del corpo (psi)
% Revolute_Joint1 = ruota sinistra (theta_L)
% Revolute_Joint2 = ruota destra (theta_R)
% z = [theta_R; theta_L; psi; w_R; w_L; w_psi].
idx_R_q   = find(endsWith(stateNames, "Revolute_Joint2.Rz.q"));
idx_L_q   = find(endsWith(stateNames, "Revolute_Joint1.Rz.q"));
idx_psi_q = find(endsWith(stateNames, "Revolute_Joint.Rz.q"));
idx_R_w   = find(endsWith(stateNames, "Revolute_Joint2.Rz.w"));
idx_L_w   = find(endsWith(stateNames, "Revolute_Joint1.Rz.w"));
idx_psi_w = find(endsWith(stateNames, "Revolute_Joint.Rz.w"));
idx_prismatic = find(contains(stateNames, "Prismatic_Joint.Pz."));

nRotStates = [numel(idx_R_q), numel(idx_L_q), numel(idx_psi_q), ...
              numel(idx_R_w), numel(idx_L_w), numel(idx_psi_w)];
if any(nRotStates ~= 1)
    error('Impossibile identificare univocamente i sei stati rotazionali dai nomi Simscape.');
end
if numel(idx_prismatic) ~= 2
    error('Attesi due stati del giunto prismatico (posizione e velocita).');
end

idx_ang = [idx_R_q; idx_L_q; idx_psi_q; idx_R_w; idx_L_w; idx_psi_w];

% Verifica che, alla linearizzazione corrente, le equazioni rotazionali
% non dipendano dalla posizione e velocita' del giunto prismatico.
translationCoupling = norm(A_raw(idx_ang, idx_prismatic), 'fro');
fprintf('Norma del coupling angolare-prismatico: %.3g\n', translationCoupling);
if translationCoupling > 1e-9
    error(['Le equazioni rotazionali dipendono dagli stati prismatici: ', ...
           'non e'' valido estrarre il sottosistema angolare a sei stati.']);
end

A_ang = A_raw(idx_ang, idx_ang);
B_ang = B_raw(idx_ang, :);

% Trasformazione nelle coordinate richieste dal professore:
% x = [theta; psi; phi; theta_dot; psi_dot; phi_dot]
% theta = (theta_R + theta_L)/2
% phi   = (wheel_radius/wheel_track)*(theta_R - theta_L)
k_yaw = wheel_radius / wheel_track;
T = [ 1/2,  1/2, 0,       0,       0, 0;
        0,    0, 1,       0,       0, 0;
     k_yaw, -k_yaw, 0,    0,       0, 0;
        0,    0, 0,     1/2,     1/2, 0;
        0,    0, 0,       0,       0, 1;
        0,    0, 0,   k_yaw,  -k_yaw, 0];

% Realizzazione angolare a sei stati. Non viene usato minreal.
A = T * A_ang / T;
B = T * B_ang;
C = eye(6);
D = zeros(6, size(B, 2));
sys = ss(A, B, C, D);
sys.StateName = {'theta'; 'psi'; 'phi'; 'theta_dot'; 'psi_dot'; 'phi_dot'};
sys.OutputName = sys.StateName;

fprintf('Ordine del sottosistema angolare: %d stati\n', order(sys));
fprintf('Dimensione di A: %dx%d\n', size(A,1), size(A,2));
fprintf('Dimensione di B: %dx%d\n\n', size(B,1), size(B,2));
disp('A nel vettore di stato richiesto:'); disp(A);
disp('B nel vettore di stato richiesto:'); disp(B);
disp('C nel vettore di stato richiesto:'); disp(C);
disp('D nel vettore di stato richiesto:'); disp(D);
disp('Autovalori a ciclo aperto del sottosistema a sei stati:'); disp(eig(A));
fprintf('Rango di controllabilita'' con una PWM: %d su 6\n', rank(ctrb(A, B)));

%% 9. Progetto LQR discreto per il bilanciamento
% Stati controllati: [theta; psi; theta_dot; psi_dot]
idx_lqr = [1 2 4 5];
A_lqr = A(idx_lqr, idx_lqr);
B_lqr = B(idx_lqr, :);
C_lqr = eye(4);
D_lqr = zeros(4, size(B_lqr, 2));
sys_lqr = ss(A_lqr, B_lqr, C_lqr, D_lqr);

rank_ctrb = rank(ctrb(A_lqr, B_lqr));
fprintf('Rango di controllabilita'' del sottosistema LQR: %d su 4\n', rank_ctrb);
if rank_ctrb < 4
    error('Il sottosistema selezionato non e'' completamente controllabile.');
end

% Pesi iniziali: dare priorita' al tilt. Da regolare dopo le simulazioni.
% Q = diag([10, 11, 10, 7]);
r_1 = 40;
r_2 = 1.2e-2;
Q = r_1*C_lqr'*C_lqr + r_2*diag([10, 11, 10, 7]);
R = 20;
max_pwm = 0.13; % max PWM for saturation

% Discretizzazione ZOH e LQR a dt
sys_lqr_d = c2d(sys_lqr, dt, 'zoh');
Ad = sys_lqr_d.A;
Bd = sys_lqr_d.B;
[K_lqr, P_lqr, poles_lqr] = dlqr(Ad, Bd, Q, R);

% Convenzione standard dlqr: u[k] = -K_lqr*x[k].
% Questi poli descrivono il loop SENZA il Unit Delay esterno del modello.
poles_check = eig(Ad - Bd*K_lqr);
disp('Guadagno LQR discreto K (u_delta[k] = -K*x_delta[k]):');
disp(K_lqr);
disp('Poli discreti restituiti da dlqr:');
disp(poles_lqr);
disp('Poli senza ritardo per la convenzione standard u=-K*x (Ad-Bd*K):');
disp(poles_check);
if any(abs(poles_check) >= 1)
    warning(['Il loop standard senza ritardo non e'' stabile: ', ...
             'verificare pesi, segni e modello.']);
end

% %% 9b. Verifica dei poli del controllore effettivamente collegato in Simulink
% % Nel modello LQR c'e' un Unit Delay tra lo stimatore e il Selector.
% % Il Gain puo' essere impostato a K_lqr oppure a -K_lqr: si legge il
% % parametro del blocco per analizzare la convenzione realmente simulata.
% model_ctrl = 'Terasic_Simscape_LQR_1PWM';
% load_system(model_ctrl);
% gainBlock = [model_ctrl '/Gain'];
% delayBlock = [model_ctrl '/Unit Delay'];
% gainExpr = strtrim(get_param(gainBlock, 'Gain'));
% delayTsExpr = strtrim(get_param(delayBlock, 'SampleTime'));
% fprintf('\n=== Verifica del loop Simulink ===\n');
% fprintf('Gain block: %s\n', gainExpr);
% fprintf('Sample time del Unit Delay: %s s (dt dello script = %.9g s)\n', ...
%         delayTsExpr, dt);
% if strcmp(delayTsExpr, 'dt')
%     delayTs = dt;
% else
%     delayTs = str2double(delayTsExpr);
% end
% if ~isnan(delayTs) && abs(delayTs - dt) > 1e-12
%     warning(['Il sample time del Unit Delay (%.9g s) differisce da dt ', ...
%              '(%.9g s): i poli aumentati calcolati qui non descrivono ', ...
%              'esattamente il ritardo del modello.'], delayTs, dt);
% end
% 
% switch gainExpr
%     case 'K_lqr'
%         K_impl = K_lqr;     % Simulink applica u[k] = K_lqr*x[k-1]
%         controlLaw = 'u[k] = K_lqr*x[k-1]';
%     case '-K_lqr'
%         K_impl = -K_lqr;    % Simulink applica u[k] = -K_lqr*x[k-1]
%         controlLaw = 'u[k] = -K_lqr*x[k-1]';
%     otherwise
%         K_impl = [];
%         controlLaw = '';
%         warning(['Il Gain block non contiene esattamente K_lqr o -K_lqr ', ...
%                  '(valore: %s). Non posso dedurre automaticamente il ', ...
%                  'guadagno effettivo; verificare manualmente il blocco.'], ...
%                  gainExpr);
% end
% 
% if ~isempty(K_impl)
%     n_lqr = size(Ad, 1);
% 
%     % Confronto dei due segni, includendo lo stesso ritardo di un campione.
%     % Legge +: u[k] = +K_lqr*x[k-1]; legge -: u[k] = -K_lqr*x[k-1].
%     A_delay_plus = [Ad,          Bd*K_lqr; ...
%                     eye(n_lqr),  zeros(n_lqr)];
%     A_delay_minus = [Ad,          -Bd*K_lqr; ...
%                      eye(n_lqr),  zeros(n_lqr)];
% 
%     poles_delay_plus = eig(A_delay_plus);
%     poles_delay_minus = eig(A_delay_minus);
%     rho_delay_plus = max(abs(poles_delay_plus));
%     rho_delay_minus = max(abs(poles_delay_minus));
% 
%     fprintf('Legge presente nel modello: %s\n', controlLaw);
%     fprintf('Gain block letto: %s\n', gainExpr);
%     fprintf('\n--- Confronto con un campione di ritardo (prima della saturazione) ---\n');
%     fprintf('Caso +K: rho = %.9g\n', rho_delay_plus);
%     disp('Poli caso +K:');
%     disp(poles_delay_plus);
%     fprintf('Caso -K: rho = %.9g\n', rho_delay_minus);
%     disp('Poli caso -K:');
%     disp(poles_delay_minus);
% 
%     if rho_delay_plus < 1
%         disp('Caso +K: stabile localmente secondo il modello lineare.');
%     else
%         disp('Caso +K: instabile localmente secondo il modello lineare.');
%     end
%     if rho_delay_minus < 1
%         disp('Caso -K: stabile localmente secondo il modello lineare.');
%     else
%         disp('Caso -K: instabile localmente secondo il modello lineare.');
%     end
% 
%     poles_delay_impl = eig([Ad, Bd*K_impl; eye(n_lqr), zeros(n_lqr)]);
%     rho_delay_impl = max(abs(poles_delay_impl));
%     fprintf('Caso effettivo nel Gain (%s): rho = %.9g\n', ...
%             gainExpr, rho_delay_impl);
% 
%     disp(['Nota: il test dei poli e'' locale e non include la saturazione ', ...
%           'PWM, i limiti/cambi di contatto o eventuali ritardi aggiuntivi.']);
% end

%% 10. Simulation
open('Terasic_Simscape_LQR_1PWM.slx')
sim('Terasic_Simscape_LQR_1PWM.slx')

%% 11. Plots

figure(1)
hold on
plot(out.tout, out.x.Data(1,:),'LineWidth',2)
plot(out.tout, out.x.Data(2,:),'LineWidth',2)
plot(out.tout, out.x.Data(3,:),'LineWidth',2)
plot(out.tout, out.x.Data(4,:),'LineWidth',2)
plot(out.tout, out.x.Data(5,:),'LineWidth',2)
plot(out.tout, out.x.Data(6,:),'LineWidth',2)
title('State evolution with LQR and one PWM', 'FontSize', 12)
xlabel('$t [s]$', 'Interpreter', 'latex')
ylabel('${x}$', 'Interpreter', 'latex')
legend('${\theta}$','${\psi}$','${\phi}$','${\dot \theta}$','${\dot \psi}$','${\dot \phi}$', 'Interpreter', 'latex','Location','best')
grid on
hold off

figure(2)
plot(out.u.Time(:,1), out.u.Data(:,1),'LineWidth',2)
title('Control law evolution with LQR and one PWM', 'FontSize', 12)
xlabel('$t [s]$', 'Interpreter', 'latex')
ylabel('${u}$', 'Interpreter', 'latex')
grid on

figure(3)
plot(out.PWM.Time(:,1), out.PWM.Data(:,1),'LineWidth',2)
title('PWM duty cycle evolution with LQR and saturation', 'FontSize', 12)
xlabel('$t [s]$', 'Interpreter', 'latex')
ylabel('${PWM}$', 'Interpreter', 'latex')
grid on

% figure(4)
% 
% % 1. Grafico principale
% plot(out.PWM.Time(:,1), out.PWM.Data(:,1), 'LineWidth', 2);
% title('PWM duty cycle evolution with LQR and saturation');
% xlabel('$t [s]$', 'Interpreter', 'latex');
% ylabel('${PWM}$', 'Interpreter', 'latex');
% grid on;
% hold on; % Mantiene il grafico attivo per eventuali aggiunte
% 
% % 2. Creazione della sotto-finestra per lo zoom (inset)
% % [sinistra, basso, larghezza, altezza] rispetto alla finestra principale (valori tra 0 e 1)
% axes('Position', [0.55, 0.52, 0.32, 0.32]); 
% 
% % 3. Ridisegno del segnale nella sotto-finestra
% plot(out.PWM.Time(:,1), out.PWM.Data(:,1), 'LineWidth', 1);
% grid on;
% 
% % 4. Impostazione dei limiti dello zoom (DA PERSONALIZZARE)
% xlim([2.0, 3.5]);     % Sostituisci con l'intervallo di tempo [t_min, t_max] che ti interessa
% % ylim([0.5, 1.0]);   % Scommenta e adatta se vuoi limitare anche l'asse y nel dettaglio
% title('none');