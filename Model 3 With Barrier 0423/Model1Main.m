clc; clear; 
close all;

% Two-Link Robot Physical Parameter Values.
L1=0.45; %length of link 1
CM1=L1/2; %center of mass of link 1
M1=0.2;%mass of link 1
J1=(1/3)*M1*L1^2;
L2=0.35; %length of link 2
CM2=L2/2; %center of mass of link 2
M2=0.6; %mass of link 2
J2=(1/3)*M2*L2^2;%moment of inertia of link 2
g=9.81; %gravity acceleration

Value = [L1;CM1;M1;J1;L2;CM2;M2;J2;g];

Units = ["m"; "m"; "kg"; 
         "kg*m^2"; "m"; "m"; 
         "kg"; "kg*m^2"; "m/s^2"];

Param = [ ...
    "Length, L1", "Center of mass, s1", "Mass, m1", ...
    "Moment of Inertia, J1", "Length, L2", ...
    "Center of mass, s2", "Mass, m2", ...
    "Moment of Inertia, J2", "Gravity, g"];

% Convert arrays to table
ParamTable = array2table(Value, "RowNames", Param);
UnitsTable = table(categorical(Units), ...
    VariableNames = "Units");
% Display parameter values.

disp([ParamTable UnitsTable])

pvstate = ParamTable.Value;

% Stage Parameters
Sf = [20; 10; 0; 0];  % Terminal Weight Matrix.
Q  = [30; 30; 0; 0];  % State Weight Matrix.
R  = [0.1; 0.1];            % Control Weighting Matrix.

p  = 50;                    % Prediction Horizon.
xf = [1; 1; 0; 0];          % Terminal State.

% Combine stage parameters into a column array.
pvcost = [xf; Sf; Q; R; p];

type("twolinkCostFcn.m")

% Define Initial State.
x0 = [0; 0; 0; 0];

% Initialize Two Link Robot plot.
figure("Color", "w")
hPlot = helperRobotEnvironment( ...
    Length1 = pvstate(1), ...
    Length2 = pvstate(5));

% Set Initial angular position and desired state. 
hPlot.Theta1 = x0(1);
hPlot.Theta2 = x0(2);
hPlot.xf = xf;

nx  = 4;
nmv = 2;
msobj = nlmpcMultistage(p,nx,nmv);

Ts = 2e-3;
msobj.Ts = Ts;

msobj.Model.StateFcn = "twolinkStateFcn";
msobj.Model.StateJacFcn = "twolinkStateJacFcn";
msobj.Model.ParameterLength = length(pvstate);

% Setting cost function for each control interval.
for k = 1:p+1
    msobj.Stages(k).CostFcn = "twolinkCostFcn";
    msobj.Stages(k).CostJacFcn = "twolinkCostJacFcn";
    msobj.Stages(k).ParameterLength = length(pvcost);
end

msobj.ManipulatedVariables(1).Min = -15;
msobj.ManipulatedVariables(1).Max =  15;
msobj.ManipulatedVariables(2).Min = -15;
msobj.ManipulatedVariables(2).Max =  15;

simdata = getSimulationData(msobj);
simdata.StateFcnParameter = pvstate;
simdata.StageParameter = repmat(pvcost, p+1, 1);

msobj.Optimization.Solver = "cgmres";

% Adjust the Stabilization Parameter 
% based on the prediction model sample time.
msobj.Optimization.SolverOptions.StabilizationParameter = 1/msobj.Ts;

% Set the solver parameters.
msobj.Optimization.SolverOptions.MaxIterations = 10;
msobj.Optimization.SolverOptions.Restart = 3;
msobj.Optimization.SolverOptions.BarrierParameter = 1e-3;
msobj.Optimization.SolverOptions.TerminationTolerance = 1e-6;

u0 = zeros(nmv,1);
[coredata, onlinedata] = getCodeGenerationData(msobj, x0, u0, StateFcnParameter=pvstate, StageParameter=repmat(pvcost, p+1, 1));

buildMEX(msobj, "nlmpcControllerMEX", coredata, onlinedata);

% Update onlinedata structure with state and stage parameters.
onlinedata.StateFcnParameter = simdata.StateFcnParameter;
onlinedata.StageParameter = simdata.StageParameter;
simdata.InitialGuess = onlinedata.InitialGuess;

[~, ~, info1] = nlmpcmove(msobj, x0, u0, onlinedata);
% % % [~, ~, info2] = nlmpcmoveCodeGeneration(coredata, x0, u0, onlinedata); 
% % % [~, ~, info3] = nlmpcControllerMEX(x0, u0, onlinedata); 

figure;
for ix = 1:nx
    subplot(3,2,ix); hold on; box on;
    plot(info1.Topt, info1.Xopt(:,ix), "-.");
    title(['State, x_' num2str(ix)]);
    xlabel("Time, s");
end

for imv = 1:nmv
    subplot(3,2,ix+imv); hold on; box on;
    plot(info1.Topt, info1.MVopt(:,imv), "-.");
    title(['Control, u_' num2str(imv)]);
    xlabel("Time, s");
end