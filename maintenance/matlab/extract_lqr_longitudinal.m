function result = extract_lqr_longitudinal(sys_raw, pitch_sign, wheel_sign)
% Extract named physical joint coordinates BEFORE minreal.
% User must establish signs and confirm state SI units and input duty scaling.
% Does not establish rolling traction or generate hardware gains.
assert(sys_raw.Ts==0,'Expected continuous sys_raw.');
assert(any(pitch_sign==[-1 1]) && any(wheel_sign==[-1 1]),'Signs must be +/-1.');
[A,B,C,D]=ssdata(sys_raw);
assert(isequal(size(A),[6 6]) && isequal(size(B),[6 1]), ...
    'Expected six named mechanical states and one common input.');
names=sys_raw.StateName;
joint_names={'Revolute_Joint','Revolute_Joint1','Revolute_Joint2'};
indices=zeros(3,2);
for joint=1:3
    for coordinate=1:2
        suffix={'q','w'};
        pattern=[joint_names{joint} '\.Rz\.' suffix{coordinate} '$'];
        hits=find(~cellfun(@isempty,regexp(names,pattern,'once')));
        assert(numel(hits)==1,'Missing/ambiguous state %s.',pattern);
        indices(joint,coordinate)=hits;
    end
end
assert(numel(unique(indices(:)))==6,'Duplicate physical-state assignment.');
T=zeros(6);
% z=[theta_relative,psi,theta_relative_dot,psi_dot,diff_angle,diff_rate]
T(1,indices(2:3,1))=wheel_sign/2;
T(2,indices(1,1))=pitch_sign;
T(3,indices(2:3,2))=wheel_sign/2;
T(4,indices(1,2))=pitch_sign;
T(5,indices(2,1))=-wheel_sign/2;
T(5,indices(3,1))= wheel_sign/2;
T(6,indices(2,2))=-wheel_sign/2;
T(6,indices(3,2))= wheel_sign/2;
Az=T*A/T;
Bz=T*B;
assert(norm(Az(1,:)-[0 0 1 0 0 0],inf)<1e-8, ...
    'Wheel coordinate kinematics inconsistent: check units.');
assert(norm(Az(2,:)-[0 0 0 1 0 0],inf)<1e-8, ...
    'Pitch coordinate kinematics inconsistent: check units.');
coupling=norm(Az(1:4,5:6),inf);
assert(coupling<1e-8*max(1,norm(Az,inf)), ...
    'Differential states affect longitudinal dynamics; cannot simply discard them.');
assert(norm(Bz(5:6),inf)<1e-8*max(1,norm(Bz,inf)), ...
    'Input is not symmetric after coordinate mapping.');
result.A_phys=Az(1:4,1:4);
result.B_phys_duty=Bz(1:4,:);
result.T=T;
result.A_modal=Az;
result.B_modal=Bz;
result.C_modal=C/T;
result.D=D;
result.joint_indices=indices;
result.pitch_sign=pitch_sign;
result.wheel_sign=wheel_sign;
result.common_from_differential_coupling=coupling;
result.state_order={'theta','psi','theta_dot','psi_dot'};
result.full_state_order={'theta','psi','rho','theta_dot','psi_dot','rho_dot'};
result.reduced_state_indices=[1 2 4 5];
% diff_angle/diff_rate above are joint differences, NOT measured yaw states.
result.input_scaling_verified=false;
result.rolling_model_verified=false;
result.note='No minreal coordinates used. SI units and input normalization need confirmation.';
disp(result);
end
