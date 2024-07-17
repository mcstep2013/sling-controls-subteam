clc; clear; 
% close all;
isFiltered=false;
isThetaProp=true;
constraintsAreAdaptive=true;

%% Two-Link Robot (Defender) Physical Parameter Values.
u0 = [0;0];
a0cart=[0;0]; %initial acceleration (needed for Kahlman Filter)

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

maxR=L1+L2;
minR=L1-L2;

%% initial/other conditions for old models, might not work anymore??

% CASE A
%Attacker requires little input to come near target
% x0 = [-0.1; 0.15; 0; 0]; %initial state
% x0Attacker=[0.9;0;pi/2;-1.8;0.9*(g/4);0];
% Duration = 0.75; % simulation duration (s)
% alpha=0;
% x0_guess  = [0.7; 0; 0; 0; 0; 0];
% x0_guess2 = [0.01; 0.35; 0; 0]; //guess thetas now
% avoidRadius=0.1; %used for constraint
% killRadius=0.04;

% CASE B
% % A neat set of ICs where the attacker is headed towards the target, but the defender gets in its way and it needs to turn around
% x0 = [-0.1; 0.15; 0; 0];
% x0Attacker=[0.9;-0.2;3*pi/4;0;0.7*(g/4);0]; 
% Duration = 1; % simulation duration (s)
% alpha=0;
% x0_guess  = [0.7; 0; 0; 0; 0; 0];
% x0_guess2 = [0.01; 0.35; 0; 0]; //guess thetas now
% avoidRadius=0.1; %used for constraint
% killRadius=0.04;

% CASE C
% %Attacker seems to be blocked by defender, and pushed down.
% x0 = [-0.1; 0.15; 0; 0];
% x0Attacker=[0.9;0.25;0;0;0;0];
% Duration = 1; % simulation duration (s)
% alpha=0;
% x0_guess  = [0.7; 0; 0; 0; 0; 0];
% x0_guess2 = [0.01; 0.168; 0; 0]; %//guess thetas now
% avoidRadius=0.1; %used for constraint
% killRadius=0.04;

% CASE D
%Attacker has same initial conditions as above, but defender is much
%further away and cannot stop it from hitting the target.
% x0 = [pi+0.2; 0; 0; 0];
% x0Attacker=[0.9;-0.1;3*pi/4;0;0.7*(g/4);0];
% Duration = 1; % simulation duration (s)
% alpha=0;
% x0_guess  = [-0.7; 0; 0; 0; 0; 0];
% x0_guess2 = [0.01; 0.35; 0; 0]; //guess thetas now
% avoidRadius=0.1; %used for constraint
% killRadius=0.04;

% CASE E
%Testing this one now
% x0=[-pi/4;0.7*pi;0;0];
% x0Attacker=[0.9;-0.1;3*pi/4;0;0.7*(g/4);0];
% Duration = 1.0; % simulation duration (s)
% alpha=0.5;
% x0_guess  = [0.4; -0.4; 0; 0; 0; 0];
% x0_guess2 = [0.01; 0.35; 0; 0]; //guess thetas now
% avoidRadius=0.1; %used for constraint
% killRadius=0.04;

% % DEFENDER BLOCKS
% % Attacker has same initial conditions as above, but defender is much
% % further away and cannot stop it from hitting the target.
% x0 = [-pi/4; 0; 0; 0];
% x0Attacker=[0.3; 0.7; -0.1; -0.3; 0.0; 0.0];
% Duration = 0.4; % simulation duration (s)
% alpha=0;
% x0_guess  = [0.6; -0.4; 0; 0; 0; 0];
% x0_guess2 = [-0.75; 0.05; 0; 0]; %//guess thetas now
% avoidRadius=0.1; %used for constraint
% killRadius=0.04;
% terminal_radius = 0.001;

% % % ATTACKER DODGES DEFENDER
% % %Attacker has same initial conditions as above, but defender is much
% % %further away and cannot stop it from hitting the target.
% x0 = [-pi/4; 0; 0; 0];
% x0Attacker=[0.3; 0.7; -0.1; -0.3; 0.0; 0.0];
% Duration = 1; % simulation duration (s)
% alpha=0;
% x0_guess  = [0.6; -0.4; 0; 0; 0; 0];
% x0_guess2 = [0.01; 0.35; 0; 0]; //guess thetas now
% avoidRadius=0.1; %used for constraint
% killRadius=0.04;
% terminal_radius = 0.001;

% %DEFENDER DETERS ATTACKER BC ALPHA
% %Attacker has same initial conditions as above, but defender is much
% %further away and cannot stop it from hitting the target.
% x0 = [-pi/4; 0; 0; 0];
% x0Attacker=[0.3; 0.7; -0.1; -0.3; 0.0; 0.0];
% Duration = 1; % simulation duration (s)
% alpha=0.5;
% x0_guess  = [0.6; -0.4; 0; 0; 0; 0];
% x0_guess2 = [0.01; 0.35; 0; 0]; //guess thetas now
% avoidRadius=0.1; %used for constraint
% killRadius=0.04;

%DEFENDER DETERS ATTACKER BC ALPHA
%Attacker has same initial conditions as above, but defender is much
%further away and cannot stop it from hitting the target.
% x0 = [-pi/4; 0; 0; 0];
% x0Attacker=[0.3; 0.7; -0.1; -0.3; 0.0; 0.0];
% Duration = 1; % simulation duration (s)
% alpha=0;
% x0_guess  = [0.6; -0.4; 0; 0; 0; 0];
% x0_guess2 = [0.01; 0.35; 0; 0]; //guess thetas now
% avoidRadius=0.1; %used for constraint
% killRadius=0.04;
% terminal_radius = 0.31;

% %TESTING NOW
% Attacker reaches target - Attacker wins
% x0 = [0; 0.4; 0; 0];
% x0Attacker=[1.3; 0; -0.1; 0; 0.0; 0.0];
% Duration = 0.4; % simulation duration (s)
% alpha=0;
% x0_guess  = [0.6; -0.4; 0; 0; 0; 0];
% x0_guess2 = [0.01; 0.35; 0; 0];
% avoidRadius=0.1; %used for constraint
% killRadius=0.04;
% terminal_radius = 0.001;


% % Case G
% x0 = [pi/4; pi/4; 0; 0];
% x0Attacker=[0.5; 0.8; pi-0.1; -0.1; 0.0; 0.0];
% Duration = 0.40; % simulation duration (s)
% alpha=0;
% x0_guess  = [0.528; 0.679; 0; 0; 0; 0];
% x0_guess2 = [0.01; -0.35; 0; 0];
% avoidRadius=0.1; %used for constraint
% killRadius=0.04;
% targetRadius=0.05;

% CASE H
%Attacker seems to be blocked by defender, and pushed down.
% x0 = [-0.5; pi/3; 0; 0];
% x0Attacker=[0.35;0.35;0;0;0;0];
% Duration = 1; % simulation duration (s)
% alpha=0.5;
% x0_guess  = [0.333; 0.369; 0; 0; 0; 0];
% x0_guess2 = [-0.488; 3.109; 0; 0]; %//guess thetas now
% avoidRadius=0.1; %used for constraint
% killRadius=0.04;

% % Case G
% %Current test
% x0 = [-0.2; 0.3; 0; 0];
% x0Attacker=[0.8; 0.1; pi-0.1; 0; 0.0; 0.0];
% Duration = 0.40; % simulation duration (s)
% alpha=0;
% x0_guess  = [0.528; 0.679; 0; 0; 0; 0];
% x0_guess2 = [0.01; -0.35; 0; 0];
% avoidRadius=0.1; %used for constraint
% killRadius=0.04;
% targetRadius=0.05;

%% Initial conditions being tested now

%A set of initial conditions that fuck everything up... Attacker dips under
%defender, the constraint is slightly slightly violated, then the attacker
%just shoots off like crazy even though thats clearly not a good idea...
x0 = [0; 0; 0; 0]; %x0 = [0; pi/4; 0; 0];
x0Attacker=[1.0; 0.0; pi/2+0.2; 0; 0.0; 0.0];
Duration = 1.0; %1.0; % simulation duration (s)
alpha=0;
x0_guess  = [0.528; 0.679; 0; 0; 0; 0];
x0_guess2 = [0.01; -0.35; 0; 0];
avoidRadius=0.1; %used for constraint
killRadius=0.04;
targetRadius=0.05;

% % %A set of initial conditions that are fine
% x0 = [0; pi/2; 0; 0]; %x0 = [0; pi/4; 0; 0];
% x0Attacker=[1.0; 0.0; pi/2+0.2; 0; 0.0; 0.0];
% Duration = 1.0; %1.0; % simulation duration (s)
% alpha=0;
% x0_guess  = [0.528; 0.679; 0; 0; 0; 0];
% x0_guess2 = [0.01; -0.35; 0; 0];
% avoidRadius=0.1; %used for constraint
% killRadius=0.04;
% targetRadius=0.05;

%% Toy rocket (attacker) Physical Parameter Values
u0Attacker=[0;0];
a0cartAttacker=[0;0]; %initial acceleration in cartesian coordinates (needed for Kahlman Filter) 
mAttacker=0.1; %mass (kg)
lAttacker=.15; %length (m) (not really used... only in calculation of IAttacker)
IAttacker=(1/12)*1.1*lAttacker^2; %moment of inertia attacker
pvstateAttacker=[mAttacker;IAttacker;g];

%% Simulation Information
Ts = 5e-3; %sample time for both the attacker and defender (easier that way)

%% Stage Parameters for Defender
Q  = [300; 300; 1; 1];  % State Weight Matrix.
Sf = 10*Q;  % Terminal Weight Matrix.
R  = [1e-10; 1e-10];            % Control Weighting Matrix.
p  = 12;                    % Prediction Horizon.
xr = [1; 1; 0; 0];          % Placeholder for reference state.
%Later in code, this is chnaged since reference state is a function of
%time.

pvcost = [xr; Sf; Q; R; p]; % Combine stage parameters into a column array.

type("twolinkCostFcn.m")

%% Stage Parameters for Attacker
%My attempt at weight adjustment
% % % Q_a=[50;50;0;1;1;1]; %state weight matrix
% % % Sf_a=10*Q_a; %Terminal state weight matrix
% % % R_a=[1e-10;1e-10]; %Control Weight matrix

% % % %From earlier versions
% % % Q_a=[3000;3000;0;1;1;0]; %state weight matrix
% % % Sf_a=10*Q_a; %Terminal state weight matrix
% % % R_a=[1e-5;1e-6]; %Control Weight matrix

%Tuning attempt 2
Q_a=[60;60;0;1;1;0.5]; %[100;100;0;1;1;2]; %[100;100;0;1;1;0]; %state weight matrix
Sf_a=10*Q_a; %Terminal state weight matrix
R_a=[0;0]; %Control Weight matrix

p_a=12; %prediction horizon
xTarget=[0;0;0;0;0;0];  %reference state=target=origin. This is what attacker is attacking.
xd=[1;1;0;0]; %placeholder for defender state, which will be changed at each stage

pvcost_a=[xTarget;Sf_a;Q_a;R_a;p_a;xd;avoidRadius];

%% Creating nlmpcMultistage object for Defender
nx  = 4; %size of state space vector
nmv = 2; %size of control input vector
msobj = nlmpcMultistage(p,nx,nmv);
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

torque_limit = 10;
msobj.ManipulatedVariables(1).Min = -2*torque_limit;
msobj.ManipulatedVariables(1).Max =  2*torque_limit;
msobj.ManipulatedVariables(2).Min = -torque_limit;
msobj.ManipulatedVariables(2).Max =  torque_limit;

thetadot_limit=1.25;
msobj.States(3).Min=-thetadot_limit;
msobj.States(3).Max=thetadot_limit;
msobj.States(4).Min=-thetadot_limit;
msobj.States(4).Max=thetadot_limit;

simdata = getSimulationData(msobj);
simdata.StateFcnParameter = pvstate;
simdata.StageParameter=repmat(pvcost,p+1,1); %This line is wrong. It is a placeholder that is corrected later in code

msobj.Optimization.Solver = "fmincon";%"cgmres";

% % % % % % Adjust the Stabilization Parameter 
% % % % % % based on the prediction model sample time.
% % % % % msobj.Optimization.SolverOptions.StabilizationParameter = 1/msobj.Ts;

% % % % % % Set the solver parameters.
% % % % % msobj.Optimization.SolverOptions.MaxIterations = 10;
% % % % % msobj.Optimization.SolverOptions.Restart = 3;
% % % % % msobj.Optimization.SolverOptions.BarrierParameter = 100;
% % % % % msobj.Optimization.SolverOptions.TerminationTolerance = 1e-6;
% % % % % msobj.Optimization.SolverOptions.StabilizationParameter=1/Ts;

%% Creating nlmpcMultistage object for Attacker
nx_a  = 6; %size of state space vector
nmv_a = 2; %size of control input (manipulated variable) vector
msobj_a = nlmpcMultistage(p_a,nx_a,nmv_a);
msobj_a.Ts = Ts;

slackVar=0.2;
%From
%https://www.mathworks.com/help/mpc/ug/specifying-constraints.html#buj077i 
% 0 — No violation allowed (hard constraint)
% 0.05 — Very small violation allowed (nearly hard)
% 0.2 — Small violation allowed (quite hard)
% 1 — average softness
% 5 — greater-than-average violation allowed (quite soft)
% 20 — large violation allowed (very soft)

msobj_a.Model.StateFcn = "attackerEOMCartesian";
msobj_a.Model.StateJacFcn = "attackerJacobiansCartesian";
msobj_a.Model.ParameterLength = length(pvstateAttacker);

% Setting cost function for each control interval.
% Also setting constraint now too
for k = 1:p_a+1
    msobj_a.Stages(k).CostFcn = "attackerCostFcnV2";
    msobj_a.Stages(k).CostJacFcn = "attackerCostJacFcnV2";
    msobj_a.Stages(k).IneqConFcn = "ineqConAvoidTipFuncV2";
    msobj_a.Stages(k).ParameterLength = length(pvcost_a);
    msobj_a.Stages(k).SlackVariableLength=length(slackVar);
end

msobj_a.ManipulatedVariables(1).Min = -0.2;
msobj_a.ManipulatedVariables(1).Max =  0.2;
msobj_a.ManipulatedVariables(2).Min = -20;
msobj_a.ManipulatedVariables(2).Max = 20;

msobj_a.States(4).Max=3;
msobj_a.States(4).Min=-3;
msobj_a.States(5).Max=3;
msobj_a.States(5).Min=-3;

simdataAttacker = getSimulationData(msobj_a);
simdataAttacker.StateFcnParameter = pvstateAttacker;
simdataAttacker.StageParameter = repmat(pvcost,p+1,1); %this line is wrong. It is a placeholder that is corrected later in the code

msobj_a.Optimization.Solver = "fmincon";%"cgmres";

% % % % % % Adjust the Stabilization Parameter 
% % % % % % based on the prediction model sample time.
% % % % % % Set the solver parameters.
% % % % % % TUNE THESE
% % % % % msobj_a.Optimization.SolverOptions.MaxIterations = 30; %10;
% % % % % msobj_a.Optimization.SolverOptions.Restart = 5; %3;
% % % % % msobj_a.Optimization.SolverOptions.BarrierParameter = 100; %1e-3;
% % % % % msobj_a.Optimization.SolverOptions.TerminationTolerance = 1e-6;
% % % % % msobj_a.Optimization.SolverOptions.StabilizationParameter = 1/msobj.Ts;

%% Some plot (remove or fix later)
% Initialize Two Link Robot plot.
figure("Color", "w")
hPlot = helperRobotEnvironmentModified( ...
    Length1 = pvstate(1), ...
    Length2 = pvstate(5));

% Set Initial angular position and desired state. 
hPlot.Theta1 = x0(1);
hPlot.Theta2 = x0(2);
hPlot.xf = x0;

%% Preparing Code generation for Defender, also some initializations
[coredata, onlinedata] = getCodeGenerationData(msobj, x0, u0, StateFcnParameter=pvstate, StageParameter=repmat(pvcost,p+1,1));

% Update onlinedata structure with state and stage parameters.
onlinedata.StateFcnParameter = simdata.StateFcnParameter;
onlinedata.StageParameter = simdata.StageParameter;
simdata.InitialGuess = onlinedata.InitialGuess;

% Store states and control for plotting purposes.
xHistory1 = x0.';
uHistory1 = u0.';

% Also storing cartesian accelerations for filtering purposes
aHistory1=a0cart.';

% Initialize current state and control.
xk = xHistory1(1,:).';
uk = uHistory1(1,:).';

%% Preparing Code generation for Attacker, also some initializations
[coredata_a, onlinedata_a] = getCodeGenerationData(msobj_a, x0Attacker, u0Attacker, StateFcnParameter=pvstateAttacker, StageParameter=repmat(pvcost_a,p_a+1,1));

% Update onlinedata structure with state and stage parameters.
onlinedata_a.StateFcnParameter = simdataAttacker.StateFcnParameter;
onlinedata_a.StageParameter = simdataAttacker.StageParameter;
simdataAttacker.InitialGuess = onlinedata_a.InitialGuess;
% % % [~, ~, info1] = nlmpcmove(msobj, x0, u0, onlinedata);

% Store states and control for plotting purposes.
xHistoryAttacker = x0Attacker.';
uHistoryAttacker = u0Attacker.';

% Also storing cartesian accelerations for filtering purposes
aHistoryAttacker=a0cartAttacker.'; %[xdd ydd] accelerations

% Initialize current state and control.
xk_a = xHistoryAttacker(1,:).';
uk_a = uHistoryAttacker(1,:).';

%% Set up EKF Object and related variables (Defender's estimation of Attacker)
rng(1);
x_true = zeros(6,Duration/Ts);
x_est  = zeros(6,Duration/Ts);
z_meas = zeros(4,Duration/Ts);
% May need to pick a smarter initial guess (guess is not of state, but of x
% xd xdd y yd ydd)
initial_covariance = diag([1, 1, 1, 1, 1, 1]);
process_noise = diag([0.005; 0.005; 0.005; 0.005; 0.005; 0.005]);
measure_noise = diag([0.1; 1e-4; 0.05; 0.025]);
% measure_noise = diag([1e-6; 1e-6; 1e-6; 1e-6]);
% set up constant acceleration EKF
filter = trackingEKF(State=x0_guess, ...
                     StateCovariance=initial_covariance, ...
                     StateTransitionFcn=@constacc, ...
                     ProcessNoise=process_noise, ...
                     MeasurementFcn= @(state) cameas(state, 'spherical'), ...
                     MeasurementNoise=measure_noise);
x_est(:,1) = filter.State;

%% Set up EKF Object and related variables (Attacker's estimation of Defender)
x_true2 = zeros(6,Duration/Ts);
x_est2  = zeros(6,Duration/Ts);
z_meas2 = zeros(4,Duration/Ts);
initial_covariance2 = diag([0.5, 0.5, 0.5, 0.5]);
process_noise2 = diag([0.0001; 0.0001; 0.0001; 0.0001]);
measure_noise2 = diag([0.075; 0.075; 0.0075; 0.0075]);
theta_est(:,1) = x0_guess2;
covariance2(:,:,1) = diag([0.1, 0.1, 0.1, 0.1]);
th1A = x0(1);
th2A = x0(2);
%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %%
%% Simulation loop
%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %%

endK=(Duration/Ts);
for k = 1:endK
    fprintf('%i of %i \n',k,endK)
    %% Defender's Estimation of Attacker
    % Get measurement vector from Attacker's True State 
    % relative to the origin (x, vx, ax, y, vy, ay):
    %  - Calculate equivalent range, range rate, and azimuth angle to the target
    %  - Add noise to the measurements to mimic sensors collecting the data 
    x_true(:,k) = [xHistoryAttacker(k,1); xHistoryAttacker(k,4); aHistoryAttacker(k,1);
               xHistoryAttacker(k,2); xHistoryAttacker(k,5); aHistoryAttacker(k,2)]; %x xd xdd y yd ydd
    if isFiltered
        z_meas(:,k) = cameas(x_true(:,k), 'spherical') + normrnd(0,measure_noise)*[1;1;1;1];
        % EKF Prediction Step
        predict(filter, Ts);
        % EKF Correction Step w/ newest measurement
        x_est(:,k) = correct(filter, z_meas(:,k));
    else
        x_est(:,k)=x_true(:,k);
    end
    x_est_vel(:,k)=x_est([1,2,4,5],k);

    % Propogate the current estimated state of the attacker forward in time
    % and determine reference state from that
    %NOTE TO SELF: Later, this is where "the alpha method" from that paper
    %will go.
    tempVar=[];
    for iter=1:p+1
        % x_est_prop = constvel(x_est_vel(:,k), iter*Ts); %this is the estimated position of attacker
        x_est_prop = x_est_vel(:,k); % TURNED OFF PROPAGATION, USE CONSTANT STATE DURING PREDICTION HORIZON
        x_est_att = x_est_prop(1); %estimated x of attacker
        y_est_att = x_est_prop(3); %estimated y of attacker
        xe=(1-alpha)*x_est_att; %estimated reference x position
        ye=(1-alpha)*y_est_att; %estimated reference y position
        % Calculate equivalent thetas for given (x,y) estimate
        dist=sqrt(xe^2+ye^2);
        if dist>maxR %if the attacker is out of reach
            angleFull=atan2(ye,xe);
            th1A=angleFull;  %set it so reference point is just pointing straight at the attacker
            th2A=0;
        elseif dist<minR %if the attacker is too close to reach
            th1A=th1A;
            th2A=th2A; %just have the defender stay where it is. Nothing it can do.
        else %reference point is where the attacker is
            th2A=acos((xe^2 + ye^2 -L1^2 -L2^2)/(2*L1*L2));
            th1A=atan2(ye,xe)-atan2((L2*sin(th2A)),(L1+L2*cos(th2A)));
            %th1 is defined counterclockwise positive wrt positive x axis
            %th2 is defined counterclockwise positive wrt the first link
        end
        tempVar=[tempVar; [th1A; th2A; 0; 0]; pvcost(5:end)]; 
        %chose 0s for theta-dots arbitrarily, since theta-dots are weighted 0 in our cost functions
    end
    
    %% Defender Control
    % Update NMPC object data w/ new target(tempVar)
    simdata.StageParameter = tempVar;
    [coredata, onlinedata] = getCodeGenerationData(msobj, x0, u0, StateFcnParameter=pvstate, StageParameter=tempVar);
    onlinedata.StageParameter = simdata.StageParameter;

    % Compute optimal control action using nlmpcmove.
    xk = xHistory1(k,:).';
    [uk, onlinedata, info] = nlmpcmove(msobj, xk, uk, onlinedata);

    %Verifying/enforcing the constraint on input
    if uk(1)<msobj.ManipulatedVariables(1).Min
        uk(1)=msobj.ManipulatedVariables(1).Min;
    elseif uk(1)>msobj.ManipulatedVariables(1).Max
        uk(1)=msobj.ManipulatedVariables(1).Max;
    end
    if uk(2)<msobj.ManipulatedVariables(2).Min
        uk(2)=msobj.ManipulatedVariables(2).Min;
    elseif uk(2)>msobj.ManipulatedVariables(2).Max
        uk(2)=msobj.ManipulatedVariables(2).Max;
    end

    %% Defender propogation to next time step
    % Simulate Two-Link Robot for the next control interval.
    ODEFUN = @(t,xk) twolinkStateFcn(xk,uk,pvstate);
    [TOUT, XOUT] = ode45(ODEFUN, [0 Ts], xHistory1(k,:));

    % Log states and control.
    xHistory1(k+1,:) = XOUT(end,:);
    uHistory1(k+1,:) = uk;

    %Also need to log acceleration (for estimation from the attacker of the defender)
    thetadotsK=ODEFUN(Ts,XOUT(end,:)'); %Using Ts as a dummy variable since our EOMs aren't time-dependent.
    aHistory1(k+1,:)=[thetadotsK(3),thetadotsK(4)];

    %% Attacker's Estimation of Defender
    % Get measurement vector from Defender's True State 
    % relative to the origin (x, vx, ax, y, vy, ay):
    %  - Add noise to the measurements to mimic sensors collecting the data 
    %NEW WAY:
    theta_true = xHistory1(k,:)'; %[xHistory1(k,1); xHistory1(k,2); xHistory1(k,3); xHistory1(k,4)]; %th1 th2 th1d th2d
    if isFiltered
        %Add noise to the measurements to mimic sensors collecting the data 
        z_meas2(:,k) = theta_true + normrnd(0,measure_noise2)*[1;1;1;1];
        Zin = z_meas2(:,k);
        Qin = process_noise2;
        Rin = measure_noise2;
        Yin = z_meas2(:,k);
        Uin = uHistory1(k,:)';
        Xin = theta_est(:,k);
        Covin = covariance2(:,:,k);
        [theta_est(:,k+1), covariance2(:,:,k+1)] = EKF_attacker_est_defender(Xin, ...
                                                                             Uin, ...
                                                                             Qin, ...
                                                                             Rin, ...
                                                                             Covin, ...
                                                                             Yin);
    else
        theta_est(:,k+1)=theta_true;
    end
    [th1,th2,th1dot,th2dot]=unpack(theta_est(:,k+1)); %assume these theta dots constant in prediction horizon
    [xcart,xdcart,~,ycart,ydcart,~]=ThetaToCart(th1,th1dot,0,th2,th2dot,0,L1,L2);
    x_est2(:,k)=[xcart;xdcart;0;ycart;ydcart;0];
    distToTip=sqrt((xHistoryAttacker(k,1)-xcart)^2 + (xHistoryAttacker(k,2)-ycart)^2);
    tipConstraintIsViolated=distToTip<avoidRadius;
    tempVar=[];
    for iter=1:p_a+1
        %doing forward euler, and assuming constant theta dots
        [xcart,xdcart,~,ycart,ydcart,~]=ThetaToCart(th1,th1dot,0,th2,th2dot,0,L1,L2);
        if tipConstraintIsViolated && constraintsAreAdaptive
            tempVar=[tempVar; [pvcost_a(1:21);xcart;ycart;xdcart;ydcart;avoidRadius]];
            % tempVar=[tempVar; [pvcost_a(1:21);xcart;ycart;xdcart;ydcart;distToTip-1e-3]];
            % tempVar=[tempVar; [pvcost_a(1:21);xcart;ycart;xdcart;ydcart;-1]];
        else
            tempVar=[tempVar; [pvcost_a(1:21);xcart;ycart;xdcart;ydcart;pvcost_a(26)]];
        end
        %Constant velocity prop (attacker estimating defender future
        %position)
        if isThetaProp
            th1=th1+th1dot*Ts;
            th2=th2+th2dot*Ts;
        end
    end
    % if k>=108
    %     pausehere=0;
    % end
    
    %% Attacker Control
   
    % Update NMPC object data w/ new target(tempVar)
    simdataAttacker.StageParameter = tempVar;
    [coredata_a, onlinedata_a] = getCodeGenerationData(msobj_a, x0Attacker, u0Attacker, StateFcnParameter=pvstateAttacker, StageParameter=tempVar);
    onlinedata.StageParameter = simdata.StageParameter;

    % Compute optimal control action using nlmpcmove.
    xk_a = xHistoryAttacker(k,:).';
    [uk_a, onlinedata_a, info2] = nlmpcmove(msobj_a, xk_a, uk_a, onlinedata_a);

    %Verifying/enforcing the constraint on input
    % % % if uk_a(1)<msobj_a.ManipulatedVariables(1).Min
    % % %     uk_a(1)=msobj_a.ManipulatedVariables(1).Min;
    % % % elseif uk_a(1)>msobj_a.ManipulatedVariables(1).Max
    % % %     uk_a(1)=msobj_a.ManipulatedVariables(1).Max;
    % % % end
    % % % if uk_a(2)<msobj_a.ManipulatedVariables(2).Min
    % % %     uk_a(2)=msobj_a.ManipulatedVariables(2).Min;
    % % % elseif uk_a(2)>msobj_a.ManipulatedVariables(2).Max
    % % %     uk_a(2)=msobj_a.ManipulatedVariables(2).Max;
    % % % end

    %% Attacker propogation to next time step
    ODEFUN = @(t,xk) attackerEOMCartesian(xk,uk_a,pvstateAttacker);
    [TOUT, XOUT] = ode45(ODEFUN, [0 Ts], xHistoryAttacker(k,:));
    Cineq(k) = ineqConAvoidTipFuncV2(k,XOUT(end,:),uk_a,0,tempVar(1:26));

    % Log states and control.
    xHistoryAttacker(k+1,:) = XOUT(end,:);
    uHistoryAttacker(k+1,:) = uk_a;

    %Also need to log acceleration (for estimation from the attacker of the defender)
    xstatedotAttacker=ODEFUN(Ts,XOUT(end,:)'); %Using Ts as a dummy variable since our EOMs aren't time-dependent.
    aHistoryAttacker(k+1,:)=[xstatedotAttacker(3),xstatedotAttacker(4)];

end

%% Plotting
timespanPlot=Ts*(0:(length(xHistory1(:,1))-1));

for k=1:length(timespanPlot)
    xHinge(k)=L1*cos(xHistory1(k,1));
    yHinge(k)=L1*sin(xHistory1(k,1));
    xDefender(k)=L1*cos(xHistory1(k,1))+L2*cos(xHistory1(k,1)+xHistory1(k,2));
    yDefender(k)=L1*sin(xHistory1(k,1))+L2*sin(xHistory1(k,1)+xHistory1(k,2));
    xRel(k)=xDefender(k)-xHistoryAttacker(k,1);
    yRel(k)=yDefender(k)-xHistoryAttacker(k,2);
    rRel(k)=(xRel(k)^2 +yRel(k)^2)^0.5;
end
%%
figure()
plot(timespanPlot,xHistory1(:,1));
hold on
plot(timespanPlot,xHistory1(:,2));
legend('\theta_1','\theta_2')
title('Defender State History')

figure()
plot(timespanPlot,xHistory1(:,3));
hold on
plot(timespanPlot,xHistory1(:,4));
legend('\omega_1','\omega_2')
title('Defender Angular Velocity History')

figure()
plot(timespanPlot,uHistory1(:,1))
hold on
plot(timespanPlot,uHistory1(:,2))
legend('u_1','u_2')
title('Defender Input History')

figure()
plot(timespanPlot,xHistoryAttacker(:,1));
hold on
plot(timespanPlot,xHistoryAttacker(:,2));
legend('X','Y')
title('Attacker State History')
ylabel('Distance (m)')
xlabel('Time (s)')

figure()
plot(timespanPlot,xHistoryAttacker(:,3));
title('Heading of attacker')
ylabel('Theta (rads)')
xlabel('Time (s)')

figure()
plot(timespanPlot,xHistoryAttacker(:,4));
hold on
plot(timespanPlot,xHistoryAttacker(:,5));
legend('Vx','Vy')
title('Attacker Velocity History')
ylabel('Velocity (m/s)')
xlabel('Time (s)')

figure()
plot(timespanPlot,uHistoryAttacker(:,1))
hold on
plot(timespanPlot,uHistoryAttacker(:,2))
legend('u_1 (torque)','u_2 (force)')
title('Attacker Input History')
xlabel('Time (s)')

figure()
plot(timespanPlot(1:end-1),Cineq)
hold on
title('Constraint: should be negative')
xlabel('Time (s)')
ylabel('C')

figure()
plot(timespanPlot,xRel)
hold on
plot(timespanPlot,yRel)
plot(timespanPlot,rRel)
legend('Relative X (cartesian)', 'Relative Y (cartesian)','Relative distance')
grid on
xlabel('Time (s)')
ylabel('Distance (m)')
title('Relative Position history')

figure()
plot(xDefender,yDefender)
hold on
plot(xHistoryAttacker(:,1),xHistoryAttacker(:,2))
legend('Defender','Attacker')
title('Positions')
xlabel('X (m)')
ylabel('Y (m)')
xlim([-1.2,1.2])
ylim([-1.2,1.2])

figure
title('Defenders Estimation of Attacker')
subplot(2,1,1)
plot(timespanPlot(1:end-1),x_true(1,:))
hold on
plot(timespanPlot(1:end-1),x_est(1,:))
legend('Truth', 'Estimate')
grid on
xlabel('Time (s)')
ylabel('X Position (m)')
ylim([-1,2])
subplot(2,1,2)
plot(timespanPlot(1:end-1),x_true(4,:))
hold on
plot(timespanPlot(1:end-1),x_est(4,:))
legend('Truth', 'Estimate')
grid on
xlabel('Time (s)')
ylabel('Y Position (m)')
ylim([-1,2])

figure
title('Defenders Estimation of Attacker')
subplot(2,1,1)
plot(timespanPlot(1:end-1),x_true2(1,:))
hold on
plot(timespanPlot(1:end-1),x_est2(1,:))
legend('Truth', 'Estimate')
grid on
xlabel('Time (s)')
ylabel('X Position (m)')
ylim([-1,2])
subplot(2,1,2)
plot(timespanPlot(1:end-1),x_true2(4,:))
hold on
plot(timespanPlot(1:end-1),x_est2(4,:))
legend('Truth', 'Estimate')
grid on
xlabel('Time (s)')
ylabel('Y Position (m)')
ylim([-1,2])

% Kill Status
kill_status = zeros(size(timespanPlot(1:end-1)));
target_status=kill_status;
for k=1:length(kill_status)
    if (rRel(k)<killRadius)
        kill_status=1;
    end
    if ((xHistoryAttacker(k,1)^2+xHistoryAttacker(k,2)^2)^0.5 < targetRadius)
        target_status=1;
    end
end
figure
plot(timespanPlot(1:end-1),kill_status); grid on;
hold on
plot(timespanPlot(1:end-1),target_status)
xlabel('Time (s)'); ylabel('Kill Status')

%%
figGif=figure();

armPlot=plot([0,xHinge(1),xDefender(1)],[0,yHinge(3),yDefender(3)],'Color','blue');
% defPlot = scatter(xDefender(1),yDefender(1),30,'Color','k','Marker','o');
hold on;
[xCirc,yCirc]=circPoints(xDefender(1),yDefender(1),avoidRadius);
circPlot=plot(xCirc,yCirc,'Color','blue');
[xKill,yKill]=circPoints(xDefender(1),yDefender(1),killRadius);
killPlot=plot(xKill,yKill,'Color','red');
[xCircGhost,yCircGhost]=circPoints(x_est2(1,1),x_est2(4,1),avoidRadius);
ghostDefenderPlot=plot(xCircGhost,yCircGhost,'Color',"#7e8dc2");
attPlot=scatter(xHistoryAttacker(1,1),xHistoryAttacker(1,2),30,'Color','red','Marker','o');
ghostAttackerPlot=scatter(x_est(1,1),x_est(4,1),30,'Color',[176, 108, 107],'Marker','o');
targetPlot=scatter(0.0, 0.0, 30,'Color',[0, 0, 0],'Marker','*');

axis equal
legend('Defender','Defender Avoid Bounds','Defender Kill Bounds','Estimated Defender','Attacker','Estimated Attacker','Target','Location','eastoutside')
xlim([-1,1.5])
ylim([-1,1])
exportgraphics(gcf,'testAnimated.gif','Append',true);
for k=2:length(xDefender(1:end-1))
    armPlot.XData=[0,xHinge(k),xDefender(k)];
    armPlot.YData=[0,yHinge(k),yDefender(k)];
    [xCirc,yCirc]=circPoints(xDefender(k),yDefender(k),avoidRadius);
    circPlot.XData=xCirc;
    circPlot.YData=yCirc;
    [xKill,yKill]=circPoints(xDefender(k),yDefender(k),killRadius);
    killPlot.XData=xKill;
    killPlot.YData=yKill;
    [xCircGhost,yCircGhost]=circPoints(x_est2(1,k),x_est2(4,k),avoidRadius);
    ghostDefenderPlot.XData=xCircGhost;
    ghostDefenderPlot.YData=yCircGhost;
    attPlot.XData=xHistoryAttacker(k,1);
    attPlot.YData=xHistoryAttacker(k,2);
    ghostAttackerPlot.XData=x_est(1,k);
    ghostAttackerPlot.YData=x_est(4,k);
    targetPlot.XData=0.0;
    targetPlot.YData=0.0;
    title(sprintf('Time=%f (s)',(k-1)*Ts));
    xlim([-1,1.5])
    ylim([-1,1])
    exportgraphics(gcf,'testAnimated.gif','Append',true);
end


function [xCirc,yCirc]=circPoints(x,y,r)
    th = 0:pi/50:2*pi;
    xCirc = r * cos(th) + x;
    yCirc = r * sin(th) + y;
end