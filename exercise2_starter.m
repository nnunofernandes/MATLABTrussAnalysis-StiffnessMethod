%% EXERCISE 2 - STUDENT STARTER for the direct stiffness method for a plane truss
% Standalone script. Base MATLAB only; no toolboxes or helper files.
% Units: N, m, Pa. Positive axial force = tension.
% Node/member labels match the supplied figures; DOFs are consecutive by node.
% First exercise: E and A are teaching assumptions (not given in its excerpt).
clear; clc; close all;

%% 1. Geometry, supports and material
nodes = [0 0; 2 0; 4 0; 2 2; 0 2];
members = [1 2; 2 3; 3 4; 2 4; 1 4; 5 4];
fixedDOFs = NaN; % TODO 0: support DOFs from the handout
loadDOF = 6;
loadValue = -30000;
E = 200e9;       % Pa
A = 0.0015;      % m^2
nNodes = size(nodes,1);
nMembers = size(members,1);
nDOF = 2*nNodes;
F = zeros(nDOF,1);
F(loadDOF) = loadValue;
U = zeros(nDOF,1); % Prescribed support displacements are zero.
if any(isnan(fixedDOFs))
    error('Complete TODO 0: identify restrained DOFs.');
end
freeDOFs = setdiff(1:nDOF,fixedDOFs);

%% 2. Assemble the global stiffness matrix
K = zeros(nDOF,nDOF);
for e = 1:nMembers
    i = members(e,1);
    j = members(e,2);
    dx = nodes(j,1)-nodes(i,1);
    dy = nodes(j,2)-nodes(i,2);
    L = NaN; % TODO 1: member length
    c = NaN; % TODO 1: x direction cosine
    s = NaN; % TODO 1: y direction cosine
    b = [-c -s c s];
    ke = NaN(4); % TODO 2: global element stiffness
    dofs = [2*i-1 2*i 2*j-1 2*j];
    K(dofs,dofs) = NaN(4); % TODO 3: accumulate element stiffness
end

if any(isnan(K(:)))
    error('Complete TODOs 1 to 3 before solving.');
end

%% 3. Solve the free displacements
Kff = K(freeDOFs,freeDOFs);
if rcond(Kff) < 1e-12
    error('Unstable or poorly conditioned model: check supports/connectivity.');
end
U(freeDOFs) = NaN; % TODO 4: solve reduced system

if any(isnan(U))
    error('Complete TODO 4 before recovering forces.');
end

%% 4. Recover reactions and member axial forces
R = K*U-F; % At free DOFs this is the numerical residual.
N = zeros(nMembers,1);
for e = 1:nMembers
    i = members(e,1);
    j = members(e,2);
    dx = nodes(j,1)-nodes(i,1);
    dy = nodes(j,2)-nodes(i,2);
    L = hypot(dx,dy);
    c = dx/L;
    s = dy/L;
    dofs = [2*i-1 2*i 2*j-1 2*j];
    N(e) = NaN; % TODO 5: recover member axial force
end
if any(isnan(N))
    error('Complete TODO 5 before viewing results.');
end
stress = N/A; % Pa

%% 5. Print results in readable units
fprintf('\nEXERCISE 2\n');
disp(table((1:nNodes).',U(1:2:end)*1e3,U(2:2:end)*1e3, ...
    'VariableNames',{'Node','ux_mm','uy_mm'}));
disp(table(fixedDOFs.',R(fixedDOFs)/1e3, ...
    'VariableNames',{'Restrained_DOF','Reaction_kN'}));
state = repmat({'zero'},nMembers,1);
state(N>1e-6) = {'tension'};
state(N<-1e-6) = {'compression'};
disp(table((1:nMembers).',N/1e3,stress/1e6,state, ...
    'VariableNames',{'Member','N_kN','Stress_MPa','State'}));
fprintf('uy at joint 2 = %.6f mm\n',U(4)*1e3);
fprintf('Force in member 5 = %.6f kN\n',N(5)/1e3);

%% 6. Verify equilibrium and the reference result
assert(norm(K-K.','fro') < 1e-10*max(1,norm(K,'fro')));
assert(norm(R(freeDOFs),inf) < 1e-6);
net = R+F;
assert(abs(sum(net(1:2:end))) < 1e-6);
assert(abs(sum(net(2:2:end))) < 1e-6);
M = sum(nodes(:,1).*net(2:2:end)-nodes(:,2).*net(1:2:end));
assert(abs(M) < 1e-6); % Moment equilibrium about the origin (N m).
assert(abs(U(4)-(-0.0009656854249492383)) < 1e-10);
assert(abs(N(5)-(-42426.40687119287)) < 1e-5);
fprintf('Equilibrium and reference checks passed.\n');

%% 7. Plot original and exaggerated deformed geometry
Umatrix = reshape(U,2,[]).';
span = max(max(nodes)-min(nodes));
maxDisp = max(sqrt(sum(Umatrix.^2,2)));
scale = 1;
if maxDisp>0
    scale = 0.12*span/maxDisp;
end
moved = nodes + scale*Umatrix;
figure('Color','w'); hold on; grid on; axis equal;
for e = 1:nMembers
    ij = members(e,:);
    h0 = plot(nodes(ij,1),nodes(ij,2),'k--','LineWidth',1);
    h1 = plot(moved(ij,1),moved(ij,2),'b-o','LineWidth',1.5);
    midpoint = mean(nodes(ij,:),1);
    text(midpoint(1),midpoint(2),sprintf('  m%d',e));
end
for i = 1:nNodes
    text(nodes(i,1),nodes(i,2),sprintf('  node %d',i));
end
legend([h0 h1],{'Original','Deformed'},'Location','best');
xlabel('x (m)'); ylabel('y (m)');
title(sprintf('Exercise 2: deformation magnified by %.0f',scale));
% This geometry plot does not draw support or load symbols: see the handout.
