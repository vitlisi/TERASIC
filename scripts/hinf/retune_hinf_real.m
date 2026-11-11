function summary = retune_hinf_real(plant_json, output_dir)
% OFFLINE ONLY. Candidate Hinf on the exported MuJoCo/encoder-filter plant.
% Does not modify the original controller, Simscape model or real binary.
% Run with Robust Control Toolbox. Each candidate still needs nonlinear tests.
assert(nargin == 2, 'Specificare JSON del plant e nuova cartella di uscita.');
assert(exist('hinfsyn','file') ~= 0, 'Serve Robust Control Toolbox.');
assert(~exist(output_dir,'dir'), 'Usare una NUOVA cartella per conservare i risultati.');
p = jsondecode(fileread(plant_json));
assert(p.schema_version == 1 && abs(p.dt_s-.01) < 1e-12);
assert(p.measurement_delay_samples == 1);
assert(isequal(string(p.input_order(:)), ["PWM_R";"PWM_L"]));
A = p.A; B = p.B; Cy = p.C_measurement; Ccost = p.C_cost;
n = size(A,1);
assert(isequal(size(A),[n,n]) && isequal(size(B),[n,2]));
assert(isequal(size(Cy),[6,n]) && isequal(size(Ccost),[6,n]));
assert(all(isfinite([A(:);B(:);Cy(:);Ccost(:)])));
% Plant augmented with the ONE external measurement delay used at runtime.
Aa = [A,zeros(n,6); Cy,zeros(6)];
Bu = [B;zeros(6,2)];
Cmeasure = [zeros(6,n),eye(6)];
state_scale = [.02/p.wheel_radius_m; deg2rad(3); deg2rad(5); ...
               .5; deg2rad(30); deg2rad(30)];
base_noise = [.002/p.wheel_radius_m;deg2rad(.2);deg2rad(.5); ...
              .05;deg2rad(2);deg2rad(2)];
cap = .20; disturbance_duty = .03;
mkdir(output_dir);
copyfile(plant_json,fullfile(output_dir,'plant_input.json'));
summary = struct('case',{},'status',{},'gamma',{},'rho',{},'order',{},'message',{});
for effort = [1,2,4]
    for derivative_noise = [1,3]
        name = sprintf('effort_%g_derivative_noise_%g',effort,derivative_noise);
        noise = base_noise; noise(4:6) = derivative_noise*noise(4:6);
        Wx = diag(1./state_scale); Wu = effort*eye(2)/cap;
        Bw = [B*disturbance_duty,zeros(n,6);zeros(6,8)];
        Cz = [Wx*Ccost,zeros(6,6);zeros(2,n+6)];
        Dzw = zeros(8,8); Dzu = [zeros(6,2);Wu];
        Dyw = [zeros(6,2),diag(noise)];
        P = ss(Aa,[Bw,Bu],[Cz;Cmeasure], ...
            [Dzw,Dzu;Dyw,zeros(6,2)],p.dt_s);
        try
            [K,CL,gamma] = hinfsyn(P,6,2);
            assert(~isempty(K) && isfinite(gamma) && isstable(CL));
            [Ak,Bk,Ck,Dk] = ssdata(K);
            Acl = [Aa+Bu*Dk*Cmeasure,Bu*Ck;Bk*Cmeasure,Ak];
            rho = max(abs(eig(Acl)));
            assert(rho < 1, 'Realizzazione interna non stabile.');
            result = struct('schema_version',1,'controller','hinf_real_offline_candidate', ...
                'Ak',Ak,'Bk',Bk,'Ck',Ck,'Dk',Dk,'dt_s',p.dt_s, ...
                'controller_initial_state',zeros(size(Ak,1),1), ...
                'measurement_delay_samples',1,'measurement_delay_initial_state',zeros(6,1), ...
                'measurement_order',{{'theta','psi','phi','theta_dot','psi_dot','phi_dot'}}, ...
                'measurement_units',{{'rad','rad','rad','rad/s','rad/s','rad/s'}}, ...
                'input_convention','estimated_state_minus_reference_at_equilibrium', ...
                'output_order',{{'PWM_R','PWM_L'}},'output_unit','normalized_duty', ...
                'output_limit',cap,'wheel_radius_m',p.wheel_radius_m, ...
                'wheel_track_m',p.wheel_track_m,'gamma',gamma, ...
                'closed_loop_spectral_radius',rho,'effort_multiplier',effort, ...
                'derivative_noise_multiplier',derivative_noise, ...
                'state_scale',state_scale,'measurement_noise_scale',noise, ...
                'plant_source','mujoco_no_slip_projection_with_50ms_encoder_filters', ...
                'validation_status','OFFLINE_ONLY_linear_no_saturation_not_real_validated');
            folder = fullfile(output_dir,name); mkdir(folder);
            save(fullfile(folder,'candidate.mat'),'K','CL','P','p','result','Acl');
            write_json(fullfile(folder,'candidate.json'),result);
            summary(end+1) = struct('case',name,'status','linear_pass_only', ...
                'gamma',gamma,'rho',rho,'order',size(Ak,1),'message','Nonlinear screening required'); %#ok<AGROW>
        catch err
            summary(end+1) = struct('case',name,'status','failed', ...
                'gamma',[],'rho',[],'order',[],'message',err.message); %#ok<AGROW>
        end
        fprintf('%s: %s\n',name,summary(end).status);
    end
end
write_json(fullfile(output_dir,'summary.json'),summary);
fprintf('Candidati SOLO offline. Nessun file sul robot modificato.\n');
end

function write_json(path,value)
fid = fopen(path,'w'); assert(fid ~= -1,'Impossibile scrivere JSON.');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(value));
end
