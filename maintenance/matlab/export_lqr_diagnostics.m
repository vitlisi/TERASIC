function export_lqr_diagnostics(output_dir)
% Run AFTER the supplied init_terasic script, in the same MATLAB session.
% Reads existing results only. Does not alter/save the Simscape model.
% The raw realization and operating point are essential; ABCD alone are not.
if nargin < 1
    output_dir = fullfile(pwd, 'lqr_export');
end
required = {'sys_raw','sys_clean','op','io','model'};
for i = 1:numel(required)
    assert(evalin('base', sprintf('exist(''%s'',''var'')',required{i})) == 1, ...
        'Run the initialization/linearization script first: missing %s.', required{i});
end
raw = evalin('base','sys_raw');
clean = evalin('base','sys_clean');
op = evalin('base','op');
io = evalin('base','io');
model = evalin('base','model');
if ~exist(output_dir,'dir'), mkdir(output_dir); end
save(fullfile(output_dir,'linearization_full.mat'),'raw','clean','op','io','model');
data.raw = describe_system(raw);
data.clean = describe_system(clean);
data.requested_controller_period_s = 0.01;
data.model = model;
data.model_file = get_param(model,'FileName');
data.operating_point = evalc('disp(op)');
data.analysis_points = evalc('disp(io)');
data.operating_states = struct('block',{},'x',{});
for i=1:numel(op.States)
    data.operating_states(i).block = op.States(i).Block;
    data.operating_states(i).x = op.States(i).x;
end
data.io_points = struct('block',{},'port',{},'type',{},'open_loop',{});
for i=1:numel(io)
    data.io_points(i).block = io(i).Block;
    data.io_points(i).port = io(i).PortNumber;
    data.io_points(i).type = io(i).Type;
    % Current IOPoint releases may encode loop opening in Type and have no
    % OpenLoop property (the user's MATLAB lists Type/Block/PortNumber only).
    data.io_points(i).open_loop = [];
    if isprop(io(i),'OpenLoop')
        data.io_points(i).open_loop = io(i).OpenLoop;
    end
end
data.operating_inputs = struct('block',{},'u',{});
for i=1:numel(op.Inputs)
    data.operating_inputs(i).block = op.Inputs(i).Block;
    data.operating_inputs(i).u = op.Inputs(i).u;
end
data.state_mapping_verified = false;
data.note = ['x of the estimator is an OUTPUT, not automatically the internal ' ...
    'state of sys_clean. No hardware gain exported.'];
text = jsonencode(data);
fid = fopen(fullfile(output_dir,'linearization_full.json'),'w');
assert(fid >= 0,'Cannot write JSON');
closer = onCleanup(@() fclose(fid));
fprintf(fid,'%s\n',text);
disp(data.raw);
disp(data.clean);
fprintf('Export saved in %s\n',output_dir);
fprintf('No controller gains or model changes were made.\n');
end

function data = describe_system(sys)
[data.A,data.B,data.C,data.D] = ssdata(sys);
data.Ts = sys.Ts;
data.StateName = sys.StateName;
data.StateUnit = sys.StateUnit;
data.InputName = sys.InputName;
data.InputUnit = sys.InputUnit;
data.OutputName = sys.OutputName;
data.OutputUnit = sys.OutputUnit;
data.poles_real = real(pole(sys));
data.poles_imag = imag(pole(sys));
end
