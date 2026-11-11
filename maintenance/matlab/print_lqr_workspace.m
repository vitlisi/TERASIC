function print_lqr_workspace()
% Text-only diagnostic for the MATLAB session after the init script.
% No model changes, no need to transfer SLX or MAT.
raw=evalin('base','sys_raw');
clean=evalin('base','sys_clean');
io=evalin('base','io');
op=evalin('base','op');
[A_raw,B_raw,C_raw,D_raw]=ssdata(raw);
fprintf('=== RAW MATRICES, Ts=%.17g, states=%d ===\n',raw.Ts,size(A_raw,1));
fprintf('A_raw = %s;\n',mat2str(A_raw,17));
fprintf('B_raw = %s;\n',mat2str(B_raw,17));
fprintf('C_raw = %s;\n',mat2str(C_raw,17));
fprintf('D_raw = %s;\n',mat2str(D_raw,17));
fprintf('=== CLEAN: Ts=%.17g, states=%d ===\n',clean.Ts,size(clean.A,1));
for k=1:numel(raw.StateName)
    fprintf('raw state %d: %s; unit=%s\n',k,raw.StateName{k},raw.StateUnit{k});
end
fprintf('=== IO ===\n');
for k=1:numel(io)
    fprintf('IO %d: type=%s; block=%s; port=%d\n', ...
        k,io(k).Type,io(k).Block,io(k).PortNumber);
end
fprintf('=== OPERATING INPUTS ===\n');
for k=1:numel(op.Inputs)
    fprintf('%s: %s\n',op.Inputs(k).Block,mat2str(op.Inputs(k).u,17));
end
end
