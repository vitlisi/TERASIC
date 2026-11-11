function snapshot = export_simscape_parameters(model, output_base)
% Esporta i parametri ATTIVI del modello aperto, senza modifiche o simulazioni.
if nargin < 2, output_base = fullfile(pwd,'simscape_parameters'); end
assert(bdIsLoaded(model), 'Aprire prima il modello e inizializzare i parametri.');
mw = get_param(model,'ModelWorkspace');
names = {'K_dlqr','dt','max_pwm','rotor_inertia','chassis_mass', ...
    'chassis_com','chassis_I_diag','chassis_I_prod','wheel_mass', ...
    'wheel_radius','wheel_track','wheel_thickness','wheel_offset_x', ...
    'joint_damping','joint_friction','motor_max_torque','V_nom', ...
    'I_stall','k_t','gear','R_int','mu_static','mu_kinematic'};
snapshot = struct('model',model,'parameters',struct(), ...
    'source',struct(),'unresolved',struct());
for j = 1:numel(names)
    name = names{j};
    try
        if hasVariable(mw,name)
            value = getVariable(mw,name);
            location = 'ModelWorkspace';
        else
            value = Simulink.data.evalinGlobal(model,name);
            location = 'global_model_workspace';
        end
        if isa(value,'Simulink.Parameter'), value = value.Value; end
        assert(isnumeric(value) && all(isfinite(value(:))), ...
            'Parametro non numerico/finito.');
        snapshot.parameters.(name) = value;
        snapshot.source.(name) = location;
    catch problem
        snapshot.unresolved.(name) = problem.message;
    end
end
snapshot.note = 'Workspace snapshot only; not proof of time-domain performance.';
save([output_base '.mat'],'snapshot','-v7');
fid = fopen([output_base '.json'],'w');
assert(fid ~= -1,'Impossibile scrivere il JSON.');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(snapshot));
fprintf('Parametri risolti: %d; non risolti: %d\n', ...
    numel(fieldnames(snapshot.parameters)),numel(fieldnames(snapshot.unresolved)));
fprintf('Esportati: %s.mat e .json\n',output_base);
end
