function model = prepare_hinf_simscape(source_slx, design_mat, destination_slx, controller_kind)
% Crea una COPIA Hinf del modello LQR 2PWM, senza avviare simulazioni.
% Il modello sorgente non viene sovrascritto. Da eseguire dopo la sintesi.
if nargin < 4, controller_kind = 'hinf'; end
assert(any(strcmp(controller_kind,{'hinf','lqr'})), 'Usare hinf oppure lqr.');
assert(isfile(source_slx) && isfile(design_mat), 'Sorgente o progetto assente.');
assert(~isfile(destination_slx), 'La destinazione esiste gia: scegliere un nuovo nome.');
s = load(design_mat,'p','Ak','Bk','Ck','Dk','result');
assert(s.result.measurement_delay_samples == 1, 'Richiesto ritardo di un campione.');
[~,model] = fileparts(destination_slx);
[~,source_name] = fileparts(source_slx);
assert(~strcmp(model,source_name), 'Usare un nome diverso dal modello sorgente.');
assert(~bdIsLoaded(source_name) && ~bdIsLoaded(model), ...
    'Chiudere prima i due modelli per evitare modifiche non salvate.');
copyfile(source_slx,destination_slx);
load_system(destination_slx);
load_system('simulink');
mw = get_param(model,'ModelWorkspace');
names = fieldnames(s.p);
for j = 1:numel(names), assignin(mw,names{j},s.p.(names{j})); end
for name = {'Ak','Bk','Ck','Dk'}
    assignin(mw,name{1},s.(name{1}));
end
assignin(mw,'hinf_x0',zeros(size(s.Ak,1),1));
assert(strcmp(get_param([model '/Gain'],'Gain'),'-K_dlqr'), ...
    'Struttura sorgente diversa da quella verificata.');
assert(strcmp(get_param([model '/Unit Delay'],'SampleTime'),'dt'), ...
    'Verificare il ritardo esterno.');
% replace_block conserva i collegamenti del blocco a un ingresso/una uscita.
if strcmp(controller_kind,'hinf')
    replaced = replace_block(model,'SearchDepth',1,'BlockType','Gain', ...
        'Gain','-K_dlqr','simulink/Discrete/Discrete State-Space','noprompt');
    assert(numel(replaced)==1, 'Atteso un solo controllore da sostituire.');
    set_param(replaced{1},'A','Ak','B','Bk','C','Ck','D','Dk', ...
        'InitialCondition','hinf_x0','SampleTime','dt');
end
% Percorsi STL portabili nella copia. I file originali rimangono intatti.
% find_system puo' includere anche il block_diagram radice: filtriamo gli
% oggetti non-blocco prima di leggere DialogParameters.
blocks = find_system(model,'LookUnderMasks','all','FollowLinks','on','Type','Block');
blocks(strcmp(blocks,model)) = [];
source_dir = fileparts(source_slx);
mesh_dirs = {fullfile(source_dir,'STL_NEW - Copia'), ...
             fullfile(fileparts(source_dir),'STL_NEW - Copia')};
for j = 1:numel(blocks)
    try
        pars = get_param(blocks{j},'DialogParameters');
    catch
        continue
    end
    if isfield(pars,'ExtGeomFileName')
        original_mesh = get_param(blocks{j},'ExtGeomFileName');
        original_mesh = strrep(original_mesh,'\','/');
        [~,stem,ext] = fileparts(original_mesh);
        candidates = [{original_mesh}, cellfun(@(d) fullfile(d,[stem ext]), ...
            mesh_dirs,'UniformOutput',false)];
        mesh = '';
        for k = 1:numel(candidates)
            if isfile(candidates{k})
                mesh = candidates{k};
                break
            end
        end
        assert(~isempty(mesh), 'STL non trovato per %s. Percorso originale: %s', ...
            blocks{j},original_mesh);
        set_param(blocks{j},'ExtGeomFileName',mesh);
    end
end
save_system(model,destination_slx);
open_system(model);
fprintf('Copia %s preparata: %s\n',controller_kind,destination_slx);
fprintf('Verificare segni dello stimatore e compilare il diagramma prima di sim.\n');
end

