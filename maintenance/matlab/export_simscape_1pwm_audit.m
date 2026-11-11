function data = export_simscape_1pwm_audit(output_dir)
% Run after init_terasic with n_PWM=1, even if its final lqr call failed.
% Read existing raw results; no trim, model changes or gain generation.
if nargin < 1
    output_dir = fullfile(fileparts(mfilename('fullpath')), ...
        '..', '..', 'logs', 'lqr', 'simscape_1pwm_20261012', 'matlab_export');
end
required = {'sys_raw','model','op','io'};
for j = 1:numel(required)
    assert(evalin('base',sprintf('exist(''%s'',''var'')',required{j})) == 1, ...
        'Missing %s: run initialization through linearization first.',required{j});
end
raw = evalin('base','sys_raw');
model = evalin('base','model');
op = evalin('base','op');
io = evalin('base','io');
assert(strcmp(model,'Terasic_Simscape_linmod_1PWM'), ...
    'This export is for the one-PWM model only.');
[data.A,data.B,data.C,data.D] = ssdata(raw);
data.Ts = raw.Ts;
data.StateName = raw.StateName;
data.StateUnit = raw.StateUnit;
data.InputName = raw.InputName;
data.InputUnit = raw.InputUnit;
data.OutputName = raw.OutputName;
data.OutputUnit = raw.OutputUnit;
data.model = model;
data.model_file = get_param(model,'FileName');
assert(size(data.B,2)==1,'Expected a single scalar linearization input.');
assert(all(isfinite([data.A(:);data.B(:);data.C(:);data.D(:)])), ...
    'Nonfinite linearization matrices.');
n = size(data.A,1);
data.controllability_rank_default_tolerance = rank(ctrb(data.A,data.B));
data.C_rank = rank(data.C);
lambda = eig(data.A);
data.poles_real = real(lambda);
data.poles_imag = imag(lambda);
data.pbh = struct('pole_real',{},'pole_imag',{},'rank',{}, ...
    'tolerance',{},'singular_values',{});
for j = 1:numel(lambda)
    M = [lambda(j)*eye(n)-data.A,data.B];
    s = svd(M);
    tol = max(size(M))*eps(max(s));
    data.pbh(j).pole_real = real(lambda(j));
    data.pbh(j).pole_imag = imag(lambda(j));
    data.pbh(j).rank = sum(s>tol);
    data.pbh(j).tolerance = tol;
    data.pbh(j).singular_values = s;
end
data.analysis_points = evalc('disp(io)');
data.operating_point = evalc('disp(op)');
data.io_points = struct('block',{},'port',{},'type',{});
for j = 1:numel(io)
    data.io_points(j).block = io(j).Block;
    data.io_points(j).port = io(j).PortNumber;
    data.io_points(j).type = io(j).Type;
end
data.operating_states = struct('block',{},'x',{});
for j = 1:numel(op.States)
    data.operating_states(j).block = op.States(j).Block;
    data.operating_states(j).x = op.States(j).x;
end
data.operating_inputs = struct('block',{},'u',{});
for j = 1:numel(op.Inputs)
    data.operating_inputs(j).block = op.Inputs(j).Block;
    data.operating_inputs(j).u = op.Inputs(j).u;
end
optional = {'opspec','opReport','linReport','dt','n_PWM'};
extra = struct();
for j = 1:numel(optional)
    if evalin('base',sprintf('exist(''%s'',''var'')',optional{j})) == 1
        extra.(optional{j}) = evalin('base',optional{j});
        data.([optional{j} '_display']) = evalc('disp(extra.(optional{j}))');
    end
end
data.workspace_parameters = struct();
parameter_names = {'R','V_nom','k_t','gear','motor_max_torque', ...
    'joint_damping','joint_friction','rotor_inertia','chassis_mass', ...
    'chassis_com','chassis_I_diag','chassis_I_prod','wheel_radius','dt','n_PWM'};
for j = 1:numel(parameter_names)
    key = parameter_names{j};
    if evalin('base',sprintf('exist(''%s'',''var'')',key)) == 1
        data.workspace_parameters.(key) = evalin('base',key);
    end
end
data.gain_expressions = struct();
for side = {'L','R'}
    block_name = ['PWM_to_tau_' side{1}];
    expression = get_param([model '/' block_name],'Gain');
    data.gain_expressions.(block_name).expression = expression;
    try
        data.gain_expressions.(block_name).current_base_workspace_value = ...
            evalin('base',expression);
    catch err
        data.gain_expressions.(block_name).evaluation_error = err.message;
    end
end
plant = [model '/Self-Balancing Robot Terasic Simulation/'];
blocks = {'static_friction_L','static_friction_R','Revolute Joint1', ...
    'Revolute Joint2','IMU'};
fields = {{'LowerValue','UpperValue','LinearizeAsGain'}, ...
    {'LowerValue','UpperValue','LinearizeAsGain'}, ...
    {'DampingCoefficient','DampingCoefficientUnits'}, ...
    {'DampingCoefficient','DampingCoefficientUnits'}, ...
    {'SenseOmega','SenseAlpha','SenseXYZDDot'}};
data.block_settings = struct('block',{},'parameters',{});
for j = 1:numel(blocks)
    data.block_settings(j).block = [plant blocks{j}];
    settings = struct();
    for k = 1:numel(fields{j})
        key = fields{j}{k};
        try
            settings.(key) = get_param([plant blocks{j}],key);
        catch err
            settings.(key) = ['UNAVAILABLE: ' err.message];
        end
    end
    data.block_settings(j).parameters = settings;
end
data.state_mapping_verified = false;
data.rolling_model_verified = false;
data.note = 'Raw results only. No LQR gains generated or validated.';
if ~exist(output_dir,'dir'), mkdir(output_dir); end
save(fullfile(output_dir,'audit_full.mat'),'raw','model','op','io','extra','data');
fid = fopen(fullfile(output_dir,'audit_full.json'),'w');
assert(fid>=0,'Cannot create JSON export.');
closer = onCleanup(@() fclose(fid));
fprintf(fid,'%s\n',jsonencode(data));
fprintf('Ts = %.17g; states = %d; inputs = %d\n',raw.Ts,n,size(data.B,2));
fprintf('A = %s;\nB = %s;\nC = %s;\nD = %s;\n', ...
    mat2str(data.A,17),mat2str(data.B,17),mat2str(data.C,17),mat2str(data.D,17));
for j = 1:n
    fprintf('State %d: %s; unit=%s\n',j,raw.StateName{j},raw.StateUnit{j});
end
fprintf('Export: %s\n',output_dir);
end
