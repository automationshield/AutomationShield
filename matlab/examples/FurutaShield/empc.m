% FURUTASHIELD EXPLICIT MPC DESIGN AND C EXPORT
%
% Linearization about the upright equilibrium from:
% https://github.com/automationshield/automationshield/wiki/FurutaShield#system-identification
%
% Requires Control System Toolbox, MPT3 and its solver dependencies.
% Run this script to regenerate ectrl_test.h in this directory.
% The header works with AutomationShield's empcSequential.h on AVR and ARM.
%
% Physical state: x = [theta0; dtheta0; theta1; dtheta1], angles in rad.
% theta1 = 0 is upright; u is arm angular acceleration [rad/s^2].
% Controller state: X = [integralError; x], MPT_DOMAIN = 5.
% Update the integrator once per sample as integralError += reference-theta0
% (without multiplying by Ts), matching the Furuta EMPC_I5 example.
% This is a local upright stabilizer; use a separate swing-up controller.
%
% This code is part of the AutomationShield hardware and software ecosystem.
% Licensed under Creative Commons Attribution-NonCommercial 4.0.

exampleDir = fileparts(mfilename('fullpath'));
addpath(fullfile(exampleDir, '..', '..')); % AutomationShield MATLAB helpers
assert(exist('LTISystem', 'class') == 8 && exist('MPCController', 'class') == 8, ...
    'Install MPT3 and its dependencies, add them to the MATLAB path, and run mpt_init.');
mpt_init;

%% Physical parameters from the wiki, converted to SI units
m1 = 6.3e-3;                 % [kg] Pendulum mass
l1 = 124.2e-3;               % [m] Pivot-to-centre-of-mass distance in the model
L0 = 99.5e-3;                % [m] Arm length
b1 = 4.26e-4;               % Viscous damping coefficient in -b1*dtheta1
g = 9.81;                   % [m/s^2] Gravitational acceleration
I1 = 0.0293;                % [kg*m^2] Literal value reported by the wiki
% The reported I1 is unusually large for this mass and length. Confirm the
% value/units for the actual apparatus before experimental controller tuning.

%% Controller settings (design choices, not identified wiki parameters)
Ts = 0.01;                  % [s] Matches FurutaShield.actuatorWrite's 10 ms update
N = 2;                      % Prediction horizon; increase with MCU memory in mind
ul = -100;                  % [rad/s^2] Lower acceleration bound
uh = 100;                   % [rad/s^2] Upper acceleration bound
% The unscaled error sum and the weak actuator coupling in the wiki model
% require a small integrator weight. Retune after identifying your apparatus.
Qmpc = diag([1e-6, 0.01, 0.001, 100, 1]); % [eI, theta0, dtheta0, theta1, dtheta1]
Rmpc = 1;

%% Continuous-time linear model from the wiki
J = I1 + m1*l1^2;
Ac = [0 1 0 0;
      0 0 0 0;
      0 0 0 1;
      0 0 g*l1*m1/J -b1/J];
Bc = [0; 1; 0; -L0*l1*m1/J];
Cc = [1 0 0 0;
      0 0 1 0];
Dc = zeros(2, 1);
modelc = ss(Ac, Bc, Cc, Dc);
modeld = c2d(modelc, Ts, 'zoh');
A = modeld.A;
B = modeld.B;
C = modeld.C;

%% Add an integrator of arm position error
% For design, reference = 0: eI(k+1) = eI(k) - theta0(k).
% During tracking, add the desired arm reference to this update.
Cr = C(1, :);
Ai = [1 -Cr; zeros(4, 1) A];
Bi = [0; B];
Ci = [zeros(2, 1) C];
assert(rank(ctrb(Ai, Bi)) == 5, 'The augmented model must be controllable.');

%% MPC with input bounds and an LQR terminal penalty, as in other examples
model = LTISystem('A', Ai, 'B', Bi, 'C', Ci, 'Ts', Ts);
model.u.min = ul;
model.u.max = uh;
model.x.penalty = QuadFunction(Qmpc);
model.u.penalty = QuadFunction(Rmpc);
model.x.with('terminalPenalty');
model.x.terminalPenalty = model.LQRPenalty;
ctrl = MPCController(model, N);
ectrl = ctrl.toExplicit();
assert(~isempty(ectrl.optimizer), 'Explicit controller generation failed.');

%% Check explicit actions against the online MPC problem
% These are numerical design checks, not hardware validation.
previousRng = rng;
rng(7);
testStates = [zeros(5, 1), diag([1, 0.1, 0.5, 0.02, 0.1])*randn(5, 50)];
rng(previousRng);
maxControlError = 0;
for k = 1:size(testStates, 2)
    [ue, feasibleExplicit] = ectrl.evaluate(testStates(:, k));
    [ui, feasibleImplicit] = ctrl.evaluate(testStates(:, k));
    assert(feasibleExplicit && feasibleImplicit, 'Infeasible validation state.');
    maxControlError = max(maxControlError, abs(ue(1)-ui(1)));
    assert(ue(1) >= ul-1e-5 && ue(1) <= uh+1e-5, 'Input bound violated.');
end
assert(maxControlError < 1e-4, 'Explicit and online MPC actions disagree.');
fprintf('Explicit/online MPC maximum difference: %.3g rad/s^2\n', maxControlError);

%% Export portable matrices using the existing AutomationShield exporter
exportFurutaHeader(ectrl, exampleDir, Ts, N, ul, uh, I1);
controllerBytes = empcMemory(ectrl, 'float');
fprintf('Regions: %d; estimated controller storage: %d bytes\n', ...
    ectrl.optimizer.Num, controllerBytes);

function exportFurutaHeader(ectrl, exampleDir, Ts, N, ul, uh, I1)
    % empcToC always writes ectrl.h in the current directory. Use a temporary
    % directory so other controllers and the caller's working directory survive.
    previousDir = pwd;
    exportDir = tempname;
    mkdir(exportDir);
    cleanup = onCleanup(@() restoreExportDirectory(previousDir, exportDir));
    cd(exportDir);
    % Keep the full horizon in MATLAB, but export only the current action.
    exportController = ectrl.copy();
    exportController.optimizer.trimFunction('primal', 1);
    empcToC(exportController, 'generic');
    body = fileread('ectrl.h');
    body = regexprep(body, '(const (?:float|int) MPT_\w+\[\]) =', ...
        '$1 FURUTA_EMPC_STORAGE =');
    preamble = sprintf([ ...
        '/* Generated by matlab/examples/FurutaShield/empc.m.\n' ...
        ' * Model: https://github.com/automationshield/automationshield/wiki/FurutaShield#system-identification\n' ...
        ' * Wiki I1 = %.10g kg*m^2; verify its units for the actual apparatus.\n' ...
        ' * X = [integralError, theta0, dtheta0, theta1, dtheta1].\n' ...
        ' * integralError += reference - theta0 (no Ts multiplier).\n' ...
        ' * Upright theta1 = 0; output is arm acceleration [rad/s^2].\n' ...
        ' * Include before empcSequential.h in one sketch translation unit.\n' ...
        ' */\n' ...
        '#ifndef FURUTASHIELD_ECTRL_TEST_H\n#define FURUTASHIELD_ECTRL_TEST_H\n\n' ...
        '#define FURUTA_EMPC_TS %.8gf\n#define FURUTA_EMPC_HORIZON %d\n' ...
        '#define FURUTA_EMPC_U_MIN %.1ff\n#define FURUTA_EMPC_U_MAX %.1ff\n\n' ...
        '#if defined(ARDUINO_ARCH_AVR) || defined(__AVR__)\n' ...
        '#include <avr/pgmspace.h>\n#define FURUTA_EMPC_STORAGE PROGMEM\n' ...
        '#else\n#define FURUTA_EMPC_STORAGE\n#endif\n\n'], I1, Ts, N, ul, uh);
    outputFile = fullfile(exampleDir, 'ectrl_test.h');
    fid = fopen(outputFile, 'w');
    assert(fid >= 0, 'Cannot write ectrl_test.h.');
    closeOutput = onCleanup(@() fclose(fid));
    fprintf(fid, '%s%s\n#undef FURUTA_EMPC_STORAGE\n#endif\n', preamble, body);
    clear closeOutput cleanup;
    fprintf('Controller header: %s\n', outputFile);
end

function restoreExportDirectory(previousDir, exportDir)
    cd(previousDir);
    % Remove only the known export file; do not recursively delete directories.
    if isfile(fullfile(exportDir, 'ectrl.h'))
        delete(fullfile(exportDir, 'ectrl.h'));
    end
    rmdir(exportDir);
end
