clc; clear; 
close all;
isFiltered=false;

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

%% A few pairs of initial/other conditions

% CASE A
%Attacker requires little input to come near target
% x0 = [-0.1; 0.15; 0; 0]; %initial state
% x0Attacker=[0.9;0;pi/2;-1.8;0.9*(g/4);0];
% Duration = 0.75; % simulation duration (s)
% alpha=0;
% x0_guess  = [0.7; 0; 0; 0; 0; 0];
% x0_guess2 = [0.8; 0; 0; 0; 0; 0];
% avoidRadius=0.04; %used for constraint

% CASE B
% % A neat set of ICs where the attacker is headed towards the target, but the defender gets in its way and it needs to turn around
% x0 = [-0.1; 0.15; 0; 0];
% x0Attacker=[0.9;-0.2;3*pi/4;0;0.7*(g/4);0]; 
% Duration = 1; % simulation duration (s)
% alpha=0;
% x0_guess  = [0.7; 0; 0; 0; 0; 0];
% x0_guess2 = [0.8; 0; 0; 0; 0; 0];
% avoidRadius=0.04; %used for constraint

% CASE C
%Attacker seems to be blocked by defender, and pushed down.
% x0 = [-0.1; 0.15; 0; 0];
% x0Attacker=[0.9;-0.1;3*pi/4;0;0.7*(g/4);0];
% Duration = 1; % simulation duration (s)
% alpha=0;
% x0_guess  = [0.7; 0; 0; 0; 0; 0];
% x0_guess2 = [0.8; 0; 0; 0; 0; 0];
% avoidRadius=0.04; %used for constraint

% CASE D
%Attacker has same initial conditions as above, but defender is much
%further away and cannot stop it from hitting the target.
% x0 = [pi+0.2; 0; 0; 0];
% x0Attacker=[0.9;-0.1;3*pi/4;0;0.7*(g/4);0];
% Duration = 1; % simulation duration (s)
% alpha=0;
% x0_guess  = [-0.7; 0; 0; 0; 0; 0];
% x0_guess2 = [0.8; 0; 0; 0; 0; 0];
% avoidRadius=0.04; %used for constraint

% CASE E
%Testing this one now
% x0=[-pi/4;0.7*pi;0;0];
% x0Attacker=[0.9;-0.1;3*pi/4;0;0.7*(g/4);0];
% Duration = 1.0; % simulation duration (s)
% alpha=0.5;
% x0_guess  = [0.4; -0.4; 0; 0; 0; 0];
% x0_guess2 = [0.8; 0; 0; 0; 0; 0];
% avoidRadius=0.04; %used for constraint

% DEFENDER BLOCKS
%Attacker has same initial conditions as above, but defender is much
%further away and cannot stop it from hitting the target.
x0 = [-pi/4; 0; 0; 0];
x0Attacker=[0.3; 0.7; -0.1; -0.3; 0.0; 0.0];
Duration = 1; % simulation duration (s)
alpha=0;
x0_guess  = [0.6; -0.4; 0; 0; 0; 0];
x0_guess2 = [0.1; 0.8; 0; 0; 0; 0];
avoidRadius=0.2; %used for constraint
terminal_radius = 0.001;

% % % ATTACKER DODGES DEFENDER
% % %Attacker has same initial conditions as above, but defender is much
% % %further away and cannot stop it from hitting the target.
% x0 = [-pi/4; 0; 0; 0];
% x0Attacker=[0.3; 0.7; -0.1; -0.3; 0.0; 0.0];
% Duration = 1; % simulation duration (s)
% alpha=0;
% x0_guess  = [0.6; -0.4; 0; 0; 0; 0];
% x0_guess2 = [0.1; 0.8; 0; 0; 0; 0];
% avoidRadius=0.05; %used for constraint
% terminal_radius = 0.001;

% %DEFENDER DETERS ATTACKER BC ALPHA
% %Attacker has same initial conditions as above, but defender is much
% %further away and cannot stop it from hitting the target.
% x0 = [-pi/4; 0; 0; 0];
% x0Attacker=[0.3; 0.7; -0.1; -0.3; 0.0; 0.0];
% Duration = 1; % simulation duration (s)
% alpha=0.5;
% x0_guess  = [0.6; -0.4; 0; 0; 0; 0];
% x0_guess2 = [0.1; 0.8; 0; 0; 0; 0];
% avoidRadius=0.05; %used for constraint

%DEFENDER DETERS ATTACKER BC ALPHA
%Attacker has same initial conditions as above, but defender is much
%further away and cannot stop it from hitting the target.
% x0 = [-pi/4; 0; 0; 0];
% x0Attacker=[0.3; 0.7; -0.1; -0.3; 0.0; 0.0];
% Duration = 1; % simulation duration (s)
% alpha=0;
% x0_guess  = [0.6; -0.4; 0; 0; 0; 0];
% x0_guess2 = [0.1; 0.8; 0; 0; 0; 0];
% avoidRadius=0.05; %used for constraint
% terminal_radius = 0.31;

%% Toy rocket (attacker) Physical Parameter Values
u0Attacker=[0;0];
a0cartAttacker=[0;0]; %initial acceleration in cartesian coordinates (needed for Kahlman Filter) 

mAttacker=0.1; %mass (kg)
lAttacker=.15; %length (m) (not really used... only in calculation of IAttacker)
IAttacker=(1/12)*0.1*lAttacker^2; %moment of inertia attacker
pvstateAttacker=[mAttacker;IAttacker;g];

%% Simulation Information
Ts = 2e-3; %sample time for both the attacker and defender (easier that way)

%% Stage Parameters for Defender
Sf = [300; 300; 0; 0];  % Terminal Weight Matrix.
Q  = [300; 300; 0; 0];  % State Weight Matrix.
R  = [0.001; 0.001];            % Control Weighting Matrix.

p  = 30;                    % Prediction Horizon.
xr = [1; 1; 0; 0];          % Placeholder for reference state.
%Later in code, this is chnaged since reference state is a function of
%time.

pvcost = [xr; Sf; Q; R; p]; % Combine stage parameters into a column array.

type("twolinkCostFcn.m")

%% Stage Parameters for Attacker
Q_a=[3000;3000;0;0;0;0]; %state weight matrix
Sf_a=Q_a; %Terminal state weight matrix
R_a=[1e-5;1e-6]; %Control Weight matrix

p_a=30; %prediction horizon
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

torque_limit = 20;
msobj.ManipulatedVariables(1).Min = -torque_limit;
msobj.ManipulatedVariables(1).Max =  torque_limit;
msobj.ManipulatedVariables(2).Min = -torque_limit;
msobj.ManipulatedVariables(2).Max =  torque_limit;

simdata = getSimulationData(msobj);
simdata.StateFcnParameter = pvstate;
simdata.StageParameter=repmat(pvcost,p+1,1); %This line is wrong. It is a placeholder that is corrected later in code

msobj.Optimization.Solver = "cgmres";

% Adjust the Stabilization Parameter 
% based on the prediction model sample time.
msobj.Optimization.SolverOptions.StabilizationParameter = 1/msobj.Ts;

% Set the solver parameters.
msobj.Optimization.SolverOptions.MaxIterations = 10;
msobj.Optimization.SolverOptions.Restart = 3;
msobj.Optimization.SolverOptions.BarrierParameter = 1e-3;
msobj.Optimization.SolverOptions.TerminationTolerance = 1e-6;

%% Creating nlmpcMultistage object for Attacker
nx_a  = 6; %size of state space vector
nmv_a = 2; %size of control input (manipulated variable) vector
msobj_a = nlmpcMultistage(p_a,nx_a,nmv_a);
msobj_a.Ts = Ts;

msobj_a.Model.StateFcn = "attackerEOMCartesian";
msobj_a.Model.StateJacFcn = "attackerJacobiansCartesian";
msobj_a.Model.ParameterLength = length(pvstateAttacker);

% Setting cost function for each control interval.
% Also setting constraint now too
for k = 1:p_a+1
    msobj_a.Stages(k).CostFcn = "attackerCostFcn";
    msobj_a.Stages(k).CostJacFcn = "attackerCostJacFcn";
    msobj_a.Stages(k).IneqConFcn = "ineqConAvoidTipFunc";
    msobj_a.Stages(k).ParameterLength = length(pvcost_a);
end

msobj_a.ManipulatedVariables(1).Min = -10;
msobj_a.ManipulatedVariables(1).Max =  10;
msobj_a.ManipulatedVariables(2).Min = -30;
msobj_a.ManipulatedVariables(2).Max = 30;

simdataAttacker = getSimulationData(msobj_a);
simdataAttacker.StateFcnParameter = pvstateAttacker;
simdataAttacker.StageParameter = repmat(pvcost,p+1,1); %this line is wrong. It is a placeholder that is corrected later in the code

msobj_a.Optimization.Solver = "cgmres";

% Adjust the Stabilization Parameter 
% based on the prediction model sample time.
msobj_a.Optimization.SolverOptions.StabilizationParameter = 1/msobj.Ts;

% Set the solver parameters.
msobj_a.Optimization.SolverOptions.MaxIterations = 10;
msobj_a.Optimization.SolverOptions.Restart = 3;
msobj_a.Optimization.SolverOptions.BarrierParameter = 1e-3;
msobj_a.Optimization.SolverOptions.TerminationTolerance = 1e-6;

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
% % % [~, ~, info1] = nlmpcmove(msobj, x0, u0, onlinedata);

% Store states and control for plotting purposes.
xHistory1 = x0.';
uHistory1 = u0.';

% Also storing cartesian accelerations for filtering purposes
aHistory1=a0cart.';

% Initialize current state and control.
xk = xHistory1(1,:).';
uk = uHistory1(1,:).';

% Initialize the accumulated elapsed time 
% for computing the optimal control action calculation.
% % % timerVal1 = 0; %I dont think this does anything?

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

% Initialize the accumulated elapsed time 
% for computing the optimal control action calculation.
% % % timerVal1 = 0; %I dont think this does anything?

%% Set up EKF Object and related variables (Defender's estimation of Attacker)
rng(1);
x_true = zeros(6,Duration/Ts);
x_est  = zeros(6,Duration/Ts);
z_meas = zeros(4,Duration/Ts);
% May need to pick a smarter initial guess (guess is not of state, but of x
% xd xdd y yd ydd)
% Random EKF inputs, may need to tune a bit
initial_covariance = diag([1, 1, 1, 1, 1, 1]);
process_noise = diag([0.005; 0.005; 0.005; 0.005; 0.005; 0.005]);
measure_noise = diag([0.05; 1e-6; 0.002; 0.001]);
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
% May need to pick a smarter initial guess (guess is not of state, but of x
% xd xdd y yd ydd)
% Random EKF inputs, may need to tune a bit
initial_covariance2 = diag([1, 1, 1, 1, 1, 1]);
process_noise2 = diag([0.01; 0.01; 0.01; 0.01; 0.01; 0.01]);
measure_noise2 = diag([0.05; 1e-6; 0.002; 0.001]);
% measure_noise2 = diag([1e-6; 1e-6; 1e-6; 1e-6]);
% set up constant acceleration EKF
filter2 = trackingEKF(State=x0_guess2, ...
                     StateCovariance=initial_covariance2, ...
                     StateTransitionFcn=@constacc, ...
                     ProcessNoise=process_noise2, ...
                     MeasurementFcn= @(state) cameas(state, 'spherical'), ...
                     MeasurementNoise=measure_noise2);
x_est2(:,1) = filter2.State;
quad_past = 0;
quad_new  = 0;
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
        z_meas(:,k) = cameas(x_true(:,k), 'spherical') + sqrt(measure_noise)*rand(4,1);
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
        x_est_prop = constvel(x_est_vel(:,k), iter*Ts); %this is the estimated position of attacker
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

    %OLD WAY:
        % % % theta_true = [xHistory1(k,1); xHistory1(k,3); xHistory1(k,2); xHistory1(k,4)]; %th1 th1d th2 th2d
        % % % a_theta_true=aHistory1(k,:);
        % % % %Converts history of derivatives of theta1 and theta2 to cartesian
        % % % [xcart,xdcart,xddcart,ycart,ydcart,yddcart]=ThetaToCart(theta_true(1),theta_true(2),a_theta_true(1),theta_true(3),theta_true(4),a_theta_true(2),L1,L2);
        % % % x_true2(:,k)=[xcart;xdcart;xddcart;ycart;ydcart;yddcart]; %true cartesian pos/vel/accel of defender
        % % % if isFiltered
        % % %     %Add noise to the measurements to mimic sensors collecting the data 
        % % %     z_meas2(:,k) = cameas(x_true2(:,k), 'spherical') + sqrt(measure_noise2)*rand(4,1);
        % % %     % EKF Prediction Step
        % % %     predict(filter2, Ts);
        % % %     % EKF Correction Step w/ newest measurement
        % % %     x_est2(:,k) = correct(filter2, z_meas2(:,k));
        % % % else
        % % %     x_est2(:,k)=x_true2(:,k);
        % % % end
        % % % 
        % % %     % Propogate the current estimated state of the defender forward in time
        % % % %This estimated propogation is stored in tempVar to be used in
        % % % %nlmpcmove as a constraint
        % % % tempVar=[];
        % % % for iter=1:p_a+1
        % % %     x_est_prop2 = constacc(x_est2(:,k), iter*Ts);
        % % %     xe2 = x_est_prop2(1);
        % % %     ye2 = x_est_prop2(4);
        % % %     xe2dot=x_est_prop2(2);
        % % %     ye2dot=x_est_prop2(5);
        % % %     tempVar=[tempVar; [pvcost_a(1:21);xe2;ye2;xe2dot;ye2dot;pvcost_a(26)]];
        % % % end

    %NEW WAY:
        theta_true = [xHistory1(k,1); xHistory1(k,3); xHistory1(k,2); xHistory1(k,4)]; %th1 th1d th2 th2d
        a_theta_true=aHistory1(k,:);
        if isFiltered
            error('This part of the code isnt written yet.')
            %I dont know how to do this... if it works w/o filtering,
            %figure this out later....
        else
            theta_est=theta_true;
        end
        [th1,th1dot,th2,th2dot]=unpack(theta_est); %assume these theta dots constant in prediction horizon
        [xcart,xdcart,~,ycart,ydcart,~]=ThetaToCart(th1,th1dot,0,th2,th2dot,0,L1,L2);
        x_est2(:,k)=[xcart;xdcart;0;ycart;ydcart;0];
        tempVar=[];
        for iter=1:p_a+1
            %doing forward euler, and assuming constant theta dots
            [xcart,xdcart,~,ycart,ydcart,~]=ThetaToCart(th1,th1dot,0,th2,th2dot,0,L1,L2);
            tempVar=[tempVar; [pvcost_a(1:21);xcart;ycart;xdcart;ydcart;pvcost_a(26)]];
            th1=th1+th1dot*Ts;
            th2=th2+th2dot*Ts;
        end
    
    %% Attacker Control
    quad_past = quad_new;
    % First Quad
    if (xHistoryAttacker(k,1) < x_est2(1,k)) && (xHistoryAttacker(k,2) < x_est2(4,k))
        quad_new = 1;
        if quad_past ~= quad_new
            msobj_a.States(1).Max = 1e5;
            msobj_a.States(1).Min = -1e5;
            msobj_a.States(2).Max = 1e5;
            msobj_a.States(2).Min = -1e5;
        end
        msobj_a.States(1).Max = x_est2(1,k)-avoidRadius;
        msobj_a.States(1).Min = -1e5;
        msobj_a.States(2).Max = x_est2(4,k)-avoidRadius;
        msobj_a.States(2).Min = -1e5;

    % Second Quad
    elseif (xHistoryAttacker(k,1) > x_est2(1,k)) && (xHistoryAttacker(k,2) < x_est2(4,k))
        quad_new = 2;
        if quad_past ~= quad_new
            msobj_a.States(1).Max = 1e5;
            msobj_a.States(1).Min = -1e5;
            msobj_a.States(2).Max = 1e5;
            msobj_a.States(2).Min = -1e5;
        end
        msobj_a.States(1).Max = 1e5;
        msobj_a.States(1).Min = x_est2(1,k)+avoidRadius;
        msobj_a.States(2).Max = x_est2(4,k)-avoidRadius;
        msobj_a.States(2).Min = -1e5;

    % Third Quad
    elseif (xHistoryAttacker(k,1) > x_est2(1,k)) && (xHistoryAttacker(k,2) > x_est2(4,k))
        quad_new = 3;
        if quad_past ~= quad_new
            msobj_a.States(1).Max = 1e5;
            msobj_a.States(1).Min = -1e5;
            msobj_a.States(2).Max = 1e5;
            msobj_a.States(2).Min = -1e5;
        end
        msobj_a.States(1).Max = 1e5;
        msobj_a.States(1).Min = x_est2(1,k)+avoidRadius;
        msobj_a.States(2).Max = 1e5;
        msobj_a.States(2).Min = x_est2(4,k)+avoidRadius; 

    % Fourth Quad
    else
        quad_new = 4;
        if quad_past ~= quad_new
            msobj_a.States(1).Max = 1e5;
            msobj_a.States(1).Min = -1e5;
            msobj_a.States(2).Max = 1e5;
            msobj_a.States(2).Min = -1e5;
        end
        msobj_a.States(1).Max = x_est2(1,k)-avoidRadius;
        msobj_a.States(1).Min = -1e5;
        msobj_a.States(2).Max = 1e5;
        msobj_a.States(2).Min = x_est2(4,k)+avoidRadius; 
    end
    % If attacker is within some radius of the target, turn off constraints
    % and go for the target
    if sqrt(xHistoryAttacker(k,1)^2 + xHistoryAttacker(k,1)^2) < terminal_radius
        msobj_a.States(1).Max = 1e5;
        msobj_a.States(1).Min = -1e5;
        msobj_a.States(2).Max = 1e5;
        msobj_a.States(2).Min = -1e5; 
    end


    % Update NMPC object data w/ new target(tempVar)
    simdataAttacker.StageParameter = tempVar;
    [coredata_a, onlinedata_a] = getCodeGenerationData(msobj_a, x0Attacker, u0Attacker, StateFcnParameter=pvstateAttacker, StageParameter=tempVar);
    onlinedata.StageParameter = simdata.StageParameter;

    % Compute optimal control action using nlmpcmove.
    xk_a = xHistoryAttacker(k,:).';
    [uk_a, onlinedata_a, info2] = nlmpcmove(msobj_a, xk_a, uk_a, onlinedata_a);

    %Verifying/enforcing the constraint on input
    if uk_a(1)<msobj_a.ManipulatedVariables(1).Min
        uk_a(1)=msobj_a.ManipulatedVariables(1).Min;
    elseif uk_a(1)>msobj_a.ManipulatedVariables(1).Max
        uk_a(1)=msobj_a.ManipulatedVariables(1).Max;
    end
    if uk_a(2)<msobj_a.ManipulatedVariables(2).Min
        uk_a(2)=msobj_a.ManipulatedVariables(2).Min;
    elseif uk_a(2)>msobj_a.ManipulatedVariables(2).Max
        uk_a(2)=msobj_a.ManipulatedVariables(2).Max;
    end

    %% Attacker propogation to next time step
    ODEFUN = @(t,xk) attackerEOMCartesian(xk,uk_a,pvstateAttacker);
    [TOUT, XOUT] = ode45(ODEFUN, [0 Ts], xHistoryAttacker(k,:));

    % Log states and control.
    xHistoryAttacker(k+1,:) = XOUT(end,:);
    uHistoryAttacker(k+1,:) = uk_a;

    %Also need to log acceleration (for estimation from the attacker of the defender)
    xstatedotAttacker=ODEFUN(Ts,XOUT(end,:)'); %Using Ts as a dummy variable since our EOMs aren't time-dependent.
    aHistoryAttacker(k+1,:)=[xstatedotAttacker(3),xstatedotAttacker(4)];

    %% Update the robot plot.
    % % % hPlot.Theta1 = xHistory1(k+1,1);
    % % % hPlot.Theta2 = xHistory1(k+1,2);
    % % % hPlot.xAtt=xHistoryAttacker(k+1,1);
    % % % hplot.yAtt=xHistoryAttacker(k+1,2);
    % % % drawnow limitrate

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

figure()
plot(timespanPlot,xHistory1(:,1));
hold on
plot(timespanPlot,xHistory1(:,2));
legend('\theta_1','\theta_2')
title('Defender State History')

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
plot(timespanPlot,uHistoryAttacker(:,1))
hold on
plot(timespanPlot,uHistoryAttacker(:,2))
legend('u_1 (torque)','u_2 (force)')
title('Attacker Input History')
xlabel('Time (s)')

% figure()
% plot(timespanPlot,xHistory1(:,1))
% hold on
% plot(timespanPlot,xrHist(1:length(timespanPlot),1))
% legend('\theta_1 of Defender','\theta_1 of Attacker')
% title('Theta 1 Comparisons (Attacker doesnt use thetas)')
% 
% figure()
% plot(timespanPlot,xHistory1(:,2))
% hold on
% plot(timespanPlot,xrHist(1:length(timespanPlot),2))
% legend('\theta_2 of Defender','\theta_2 of Attacker')
% title('Theta 2 Comparisons (Attacker doesnt use thetas)')

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

%%
figGif=figure();

armPlot=plot([0,xHinge(1),xDefender(1)],[0,yHinge(3),yDefender(3)],'Color','blue');
% defPlot = scatter(xDefender(1),yDefender(1),30,'Color','k','Marker','o');
hold on;
[xCirc,yCirc]=circPoints(xDefender(1),yDefender(1),avoidRadius);
circPlot=plot(xCirc,yCirc,'Color','blue');
attPlot=scatter(xHistoryAttacker(1,1),xHistoryAttacker(1,2),30,'Color','red','Marker','o');
[xCircGhost,yCircGhost]=circPoints(x_est2(1,1),x_est2(4,1),avoidRadius);
ghostDefenderPlot=plot(xCircGhost,yCircGhost,'Color',"#7e8dc2");
ghostAttackerPlot=scatter(x_est(1,1),x_est(4,1),30,'Color',[176, 108, 107],'Marker','o');
axis equal
legend('Defender','Defender Tip Bounds','Attacker','Estimated Defender','Estimated Attacker', 'Location','eastoutside')
xlim([-1,1])
ylim([-1,1])
exportgraphics(gcf,'testAnimated.gif','Append',true);
for k=2:length(xDefender(1:end-1))
    [xCirc,yCirc]=circPoints(xDefender(k),yDefender(k),avoidRadius);
    circPlot.XData=xCirc;
    circPlot.YData=yCirc;
    % defPlot.XData=xDefender(k);
    % defPlot.YData=yDefender(k);
    armPlot.XData=[0,xHinge(k),xDefender(k)];
    armPlot.YData=[0,yHinge(k),yDefender(k)];
    attPlot.XData=xHistoryAttacker(k,1);
    attPlot.YData=xHistoryAttacker(k,2);
    [xCircGhost,yCircGhost]=circPoints(x_est2(1,k),x_est2(4,k),avoidRadius);
    ghostDefenderPlot.XData=xCircGhost;
    ghostDefenderPlot.YData=yCircGhost;
    ghostAttackerPlot.XData=x_est(1,k);
    ghostAttackerPlot.YData=x_est(4,k);
    title(sprintf('Time=%f (s)',(k-1)*Ts));
    xlim([-1,1])
    ylim([-1,1])
    exportgraphics(gcf,'testAnimated.gif','Append',true);
end


function [xCirc,yCirc]=circPoints(x,y,r)
    th = 0:pi/50:2*pi;
    xCirc = r * cos(th) + x;
    yCirc = r * sin(th) + y;
end