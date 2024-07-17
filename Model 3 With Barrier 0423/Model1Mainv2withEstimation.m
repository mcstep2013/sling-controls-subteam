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

% Simulation duration in seconds.
Duration = 2;
%sample time
Ts = 2e-3;

% Stage Parameters
Sf = [30; 30; 0; 0];  % Terminal Weight Matrix.
Q  = [30; 30; 0; 0];  % State Weight Matrix.
R  = [0.1; 0.1];            % Control Weighting Matrix.

p  = 50;                    % Prediction Horizon.
xr = [1; 1; 0; 0];          % Reference State. Will make a function of time

% Combine stage parameters into a column array.
pvcost = [xr; Sf; Q; R; p];

type("twolinkCostFcn.m")

% Define Initial State.
x0 = [-0.1; 0.15; 0; 0];

%% Attacker
%projectile under projectile motion
x0A=0.9;
vx0A=-1.8;
y0A=0;
vy0A=g/4;
xFull0A=[x0A;y0A;vx0A;vy0A];
for k=1:(Duration/Ts)+1
    xA(k)=x0A+(vx0A*((k-1)*Ts));
    yA(k)=y0A+(vy0A*((k-1)*Ts))-(0.5*g*((k-1)*Ts)^2);
    vxA(k)=vx0A;
    vyA(k)=vy0A-(g*(k-1)*Ts);
    axA(k)=0;
    ayA(k)=-g;
end
xrefCart=[xA',yA',vxA',vyA'];
maxR=L1+L2;
minR=L1-L2;
for k=1:(Duration/Ts)+1
    dist=sqrt(xrefCart(k,1)^2+xrefCart(k,2)^2);
    if dist>maxR %if the attacker is out of reach
        angleFull=atan2(yA(k),xA(k));
        th1A(k,1)=angleFull;  %set it so reference point is just pointing straight at the attacker
        th2A(k,1)=0;
    elseif dist<minR %if the attacker is too close to reach
        th1A(k,1)=th1A(k-1);
        th2A(k)=th2A(k-1); %just have the defender stay where it is. Nothing it can do.
    else %reference point is where the attacker is
        th2A(k,1)=acos((xA(k)^2 + yA(k)^2 -L1^2 -L2^2)/(2*L1*L2));
        th1A(k,1)=atan2(yA(k),xA(k))-atan2((L2*sin(th2A(k))),(L1+L2*cos(th2A(k))));
        %th1 is defined counterclockwise positive wrt positive x axis
        %th2 is defined counterclockwise positive wrt the first link
    end
end
xrHist=[th1A,th2A,zeros((Duration/Ts)+1,2)]; %choosing velocity zero since i dont use velocity in cost anyways
xrHist=[xrHist;repmat(xrHist(end,:),51,1)];

%%

% Initialize Two Link Robot plot.
figure("Color", "w")
hPlot = helperRobotEnvironment( ...
    Length1 = pvstate(1), ...
    Length2 = pvstate(5));

% Set Initial angular position and desired state. 
hPlot.Theta1 = x0(1);
hPlot.Theta2 = x0(2);
hPlot.xf = xr;

nx  = 4;
nmv = 2;
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

msobj.ManipulatedVariables(1).Min = -17.5;
msobj.ManipulatedVariables(1).Max =  17.5;
msobj.ManipulatedVariables(2).Min = -17.5;
msobj.ManipulatedVariables(2).Max =  17.5;

simdata = getSimulationData(msobj);
simdata.StateFcnParameter = pvstate;
tempVar=[];
for k=1:p+1
    tempVar=[tempVar; xrHist(k,:)';pvcost(5:end)];
end
simdata.StageParameter = tempVar;

msobj.Optimization.Solver = "cgmres";

% Adjust the Stabilization Parameter 
% based on the prediction model sample time.
msobj.Optimization.SolverOptions.StabilizationParameter = 1/msobj.Ts;

% Set the solver parameters.
msobj.Optimization.SolverOptions.MaxIterations = 10;
msobj.Optimization.SolverOptions.Restart = 3;
msobj.Optimization.SolverOptions.BarrierParameter = 1e-3;
msobj.Optimization.SolverOptions.TerminationTolerance = 1e-6;

%%
u0 = zeros(nmv,1);
[coredata, onlinedata] = getCodeGenerationData(msobj, x0, u0, StateFcnParameter=pvstate, StageParameter=tempVar);

% Update onlinedata structure with state and stage parameters.
onlinedata.StateFcnParameter = simdata.StateFcnParameter;
onlinedata.StageParameter = simdata.StageParameter;
simdata.InitialGuess = onlinedata.InitialGuess;
[~, ~, info1] = nlmpcmove(msobj, x0, u0, onlinedata);

% Store states and control for plotting purposes.
xHistory1 = x0.';
uHistory1 = u0.';

% Initialize current state and control.
xk = xHistory1(1,:).';
uk = uHistory1(1,:).';

% Initialize the accumulated elapsed time 
% for computing the optimal control action calculation.
timerVal1 = 0;

% Set up EKF Object and related variables
rng(1);
x_true = zeros(6,Duration/Ts);
x_est  = zeros(6,Duration/Ts);
z_meas = zeros(4,Duration/Ts);
% May need to pick a smarter initial guess
x0_guess = [0; 0; 0; 45; 0; 0];
% Random EKF inputs, may need to tune a bit
initial_covariance = diag([1, 1, 1, 1, 1, 1]);
process_noise = diag([0.01; 0.01; 0.01; 0.01; 0.01; 0.01]);
measure_noise = diag([0.5; 1e-6; 0.2; 0.1]);
% set up constant acceleration EKF
filter = trackingEKF(State=x0_guess, ...
                     StateCovariance=initial_covariance, ...
                     StateTransitionFcn=@constacc, ...
                     ProcessNoise=process_noise, ...
                     MeasurementFcn= @(state) cameas(state, 'spherical'), ...
                     MeasurementNoise=measure_noise);
x_est(:,1) = filter.State;

%% Simulation loop
for k = 1:(Duration/Ts)

    % Get measurement vector from Attacker's True State 
    % relative to the origin (x, vx, ax, y, vy, ay):
    %  - Calculate equivalent range, range rate, and azimuth angle to the target
    %  - Add noise to the measurements to mimic sensors collecting the data 
    x_true(:,k) = [xA(k); vxA(k); 0; yA(k); vyA(k); -g];
    z_meas(:,k) = cameas(x_true(:,k), 'spherical') + sqrt(measure_noise)*rand(4,1);
    % EKF Prediction Step
    predict(filter, Ts);
    % EKF Correction Step w/ newest measurement
    x_est(:,k) = correct(filter, z_meas(:,k));

    tempVar=[];
    for iter=1:p+1
        % Propogate the current estimated state forward in time
        x_est_prop = constacc(x_est(:,k), iter*Ts);
        xe = x_est_prop(1);
        ye = x_est_prop(4);
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
    end
    
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

    % Simulate Two-Link Robot for the next control interval.
    ODEFUN = @(t,xk) twolinkStateFcn(xk,uk,pvstate);
    [TOUT, XOUT] = ode45(ODEFUN, [0 Ts], xHistory1(k,:));

    % Log states and control.
    xHistory1(k+1,:) = XOUT(end,:);
    uHistory1(k+1,:) = uk;
    
    % Update two-link robot plot.
    hPlot.Theta1 = xHistory1(k+1,1);
    hPlot.Theta2 = xHistory1(k+1,2);
    drawnow limitrate

end

%% Plotting
timespanPlot=Ts*(0:(length(xHistory1(:,1))-1));

for k=1:length(timespanPlot)
    xDefender(k)=L1*cos(xHistory1(k,1))+L2*cos(xHistory1(k,1)+xHistory1(k,2));
    yDefender(k)=L1*sin(xHistory1(k,1))+L2*sin(xHistory1(k,1)+xHistory1(k,2));
    xRel(k)=xDefender(k)-xA(k);
    yRel(k)=yDefender(k)-yA(k);
end

figure()
plot(timespanPlot,xHistory1(:,1));
hold on
plot(timespanPlot,xHistory1(:,2));
legend('x_1','x_2')

figure()
plot(timespanPlot,uHistory1(:,1))
hold on
plot(timespanPlot,uHistory1(:,2))
legend('u_1','u_2')

figure()
plot(timespanPlot,xHistory1(:,1))
hold on
plot(timespanPlot,xrHist(1:length(timespanPlot),1))
legend('\theta_1 of Defender','\theta_1 of Attacker')

figure()
plot(timespanPlot,xHistory1(:,2))
hold on
plot(timespanPlot,xrHist(1:length(timespanPlot),2))
legend('\theta_2 of Defender','\theta_2 of Attacker')

figure()
plot(timespanPlot,xRel)
hold on
plot(timespanPlot,yRel)
legend('Relative X (cartesian)', 'Relative Y (cartesian)')
grid on
xlabel('Time (s)')
ylabel('Distance (m)')

figure
subplot(2,1,1)
plot(timespanPlot(1:end-1),x_true(1,:))
hold on
plot(timespanPlot(1:end-1),x_est(1,:))
legend('Truth', 'Estimate')
grid on
xlabel('Time (s)')
ylabel('X Position (m)')
subplot(2,1,2)
plot(timespanPlot(1:end-1),x_true(4,:))
hold on
plot(timespanPlot(1:end-1),x_est(4,:))
legend('Truth', 'Estimate')
grid on
xlabel('Time (s)')
ylabel('Y Position (m)')


