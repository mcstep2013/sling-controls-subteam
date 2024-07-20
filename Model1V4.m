clear;
clc;
% close all;

addpath(genpath('utilities'));  % Add folder and subfolders
addpath(genpath('saved data'));  % Add folder and subfolders

%% Problem setup and constants etc
mu_moon = 4902.8005821478;  % Both in km^3 / s^2
mu_earth = 398600.4415;

l_star = 384400;  %Length used to nondimensionalize, also equal to the SMA of the Moon
t_star = sqrt(l_star^3 / (mu_moon+mu_earth)); % Divide by 86400 to get time in days
n=1/t_star; %mean motion of earth-moon system, and angular rate of synodic frame
v_star = l_star / t_star;

mu = mu_moon / (mu_earth + mu_moon);  % mu value for the Earth-Moon system
mu_oom = round(log10(mu)); mu_SF_known = 8;  % We know 8 Earth sigfigs, 9 Moon sigfigs

% nondimensional mu rounded to appropriate significance
mu = round(mu*10^(mu_SF_known-mu_oom-1)) / 10^(mu_SF_known-mu_oom-1);

%% Calculate libration point characteristics
[x_L1, gam_L1] = calculate_Li(1, mu, 1e-9);
[x_L2, gam_L2] = calculate_Li(2, mu, 1e-9);
[x_L3, gam_L3] = calculate_Li(3, mu, 1e-9);

JC_1 = 2 * PsuPot([x_L1, 0, 0], mu);
JC_2 = 2 * PsuPot([x_L2, 0, 0], mu);
JC_3 = 2 * PsuPot([x_L3, 0, 0], mu);
JC_4 = 2 * PsuPot([1/2-mu, sqrt(3)/2, 0], mu);
JC_5 = 2 * PsuPot([1/2-mu, -sqrt(3)/2, 0], mu);

JC_Li_set = [JC_1, JC_2, JC_3, JC_4];
Lag_X = [x_L1, x_L2, x_L3];

%% Definition of option structures
opts = odeset("RelTol",1e-12,"AbsTol",1e-12);

%% Initial conditions
load('saved data\Halos\EM_L1N_92JC.mat')
% Start from a data structure that contains initial conditions and period
tetherCM_initial_state = L1N_92JC.ic;
T = L1N_92JC.TIP;
% Perform pseudo-arclength continuation

% Define tether spin parameters 
%NOTE: I HAVE NO IDEA WHAT THESE ARE
tether_start_time = 0.3133;  % Collision starts at T = 1.2
tip_phase = 0.572;

% load("halo_catch_unstable_manif_IC.mat")
load('saved data\tether_catch_optimal_control_data\min_energy_plusminus_twentyfour_hours.mat')

% These chosen IC's give the tether a fair shot at catching
SC_initial_state = min_e_traj(1).initial_state;

%% Still need initial state for orentation and length of tether
nominalLength=200; %dimensional initial length of tether
l0=nominalLength/l_star;
alpha_til = 710.68;  % Back calculated from some old simulation of nonextendable tether
alpha0=0; %CHOSEN ARBITARILY FOR NOW... PUT MORE THOUGHT INTO THIS LATER?
alphadot0=(alpha_til+1)*n; %this is what I thought it should be, but its not spinning around very much so idk ill increase it
alphadot0=alphadot0*500;

%% Full initial conditions
x0=[SC_initial_state; tetherCM_initial_state; l0; alpha0; alphadot0];
u0=[0;0;0;0];

%% pvcost, Tuning for control
timedimensional=-10000; %placeholder, to be changed each time through the loop
p=10; %prediction horizon, control horizon
Q=[1;1;1;0.005;0.005;0.005]; %state weight matrix
Sf=Q*10; %terminal state weight matrix
R=[1;1;1;0.1]*0.0001;
pvcost=[mu;n;timedimensional;p;Q;Sf;R];

Ts=0.01; %sampling time (nondimensional units)

%% Creating objects for nmpc
nx=15;
nu=4;
msobj = nlmpcMultistage(p,nx,nu);
msobj.Ts=Ts;
pvstate=[mu]; %only one thing needed, can add others later if needed

msobj.Model.StateFcn = "stateFuncExtendable";
msobj.Model.StateJacFcn = "ExtendableTetherJacobians";
msobj.Model.ParameterLength = length(pvstate);

for k=1:p+1
    msobj.Stages(k).CostFcn = "costFuncExtendable";
    msobj.Stages(k).CostJacFcn = "ExtendableTetherCostJacobians";
    msobj.Stages(k).ParameterLength = length(pvcost);
end

accel_limit = 50;
msobj.ManipulatedVariables(1).Min = -accel_limit;
msobj.ManipulatedVariables(1).Max =  accel_limit;
msobj.ManipulatedVariables(2).Min = -accel_limit;
msobj.ManipulatedVariables(2).Max =  accel_limit;
msobj.ManipulatedVariables(3).Min = -accel_limit;
msobj.ManipulatedVariables(3).Max =  accel_limit;

lengthROC_limit=2e-3;
msobj.ManipulatedVariables(4).Min=-lengthROC_limit;
msobj.ManipulatedVariables(4).Max=lengthROC_limit;

msobj.Optimization.Solver = "fmincon";

simdata = getSimulationData(msobj);
simdata.StateFcnParameter = pvstate;
simdata.StageParameter=repmat(pvcost,p+1,1); %This line is wrong. 
% It is a placeholder that is corrected later in code. The actual parameters for
% the cost function need to be updated each at each step, since this is an 
% nlmp multistage. As of now, this is only useful for one element (the 
% nondimensional time), but there may be more in the future.

%% Some initializations
[coredata, onlinedata] = getCodeGenerationData(msobj, x0, u0, StateFcnParameter=pvstate, StageParameter=repmat(pvcost,p+1,1));

% Update onlinedata structure with state and stage parameters.
onlinedata.StateFcnParameter = simdata.StateFcnParameter;
onlinedata.StageParameter = simdata.StageParameter;
simdata.InitialGuess = onlinedata.InitialGuess;

% Store states and control for plotting purposes.
xHistory1 = x0.';
uHistory1 = u0.';

% Initialize current state and control.
xk = xHistory1(1,:).';
uk = uHistory1(1,:).';

%% Debug test
%Using initial (ie 0) control input, see what happens
ODETEST = @(t,xk) stateFuncExtendable(xk,u0,pvstate);
[Ttest, Xtest] = ode45(ODETEST, [0 T], xHistory1(1,:));

Ttestdim=Ttest*t_star;
for ind=1:length(Ttest)
    xtesttip(:,ind)=getTetherTipState(Xtest(ind,:)',u0, [mu,n,Ttestdim(ind)]);
end
xtesttip=xtesttip';

figure()
plot3(Xtest(:,7),Xtest(:,8),Xtest(:,9));
hold on
plot3(xtesttip(:,1),xtesttip(:,2),xtesttip(:,3));
title('COM of tether almost in halo orbit, eh close enough')
legend('Center of Mass','Tip')
axis equal

%% Duration
Duration = T/2; %in nondimensionalized time

%% Simulation loop
endK=floor(Duration/Ts);
for k=1:endK
    fprintf('%i of %i\n',k,endK)
    tempVar=[];
    for iter=1:p+1
        currentTimeDimensional=(k+iter-2)*Ts*t_star;
        tempVar=[tempVar; pvcost(1:2); currentTimeDimensional; pvcost(4:end)];
    end

    %% Compute Control
    % Update NMPC object data w/ new target(tempVar)
    simdata.StageParameter = tempVar;
    [coredata, onlinedata] = getCodeGenerationData(msobj, x0, u0, StateFcnParameter=pvstate, StageParameter=tempVar);
    onlinedata.StageParameter = simdata.StageParameter;

    % Compute optimal control action using nlmpcmove.
    xk = xHistory1(k,:).';
    [uk, onlinedata, info] = nlmpcmove(msobj, xk, uk, onlinedata);

    %TO DO: enforce input limits here

    %% Propogation
    ODEFUN = @(t,xk) stateFuncExtendable(xk,uk,pvstate);
    [TOUT, XOUT] = ode45(ODEFUN, [0 Ts], xHistory1(k,:));

    % Log states and control.
    xHistory1(k+1,:) = XOUT(end,:);
    uHistory1(k,:) = uk;

end

%% Plotting
for k=1:endK
    timedimensional=(k-1)*Ts*t_star;
    xtip(:,k)=getTetherTipState(xHistory1(k,:)',uHistory1(k,:)',[mu,n,timedimensional]);
end
xtip=xtip';

figure()
plot3(xtip(:,1),xtip(:,2),xtip(:,3));
hold on
plot3(xHistory1(:,1),xHistory1(:,2),xHistory1(:,3));
plot3(xHistory1(:,7),xHistory1(:,8),xHistory1(:,9));
legend('Tether Tip','Spacecraft','Tether COM')
axis equal

xrel=xHistory1(1:endK,1:6)-xtip(1:endK,:);
for i=1:length(xrel(:,1))
    reldist(i)=norm(xrel(i,1:3));
    relvel(i)=norm(xrel(i,4:6));
end

figure()
plot(1:length(reldist),reldist)

figure()
plot(1:length(reldist),relvel)


