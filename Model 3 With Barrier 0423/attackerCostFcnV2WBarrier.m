function L = attackerCostFcnV2WBarrier(stage, x, u, e, pvCost)
% twolinkCostFcn
%   L = twolinkCostFcn(stage, x, u, pvcost)
%   
%   This function computes the objective cost for the two-link robot
%   manipulator example.
%
%   Inputs:
%       - stage     Current stage of the Multistage MPC problem
%       - x         Current state vector
%       - u         Control action vector at the previous interval
%       - pvcost    Vector containing various stage parameters
%
%   Output:
%       - L:        Cost value calculated by the cost function.

% Copyright 2023 The MathWorks, Inc.

% Extract Target State
xt = pvCost(1:6);

% Extract and Initialize Wiegthing Matrices
Sf = diag(pvCost(7:12));                 % Terminal Weight Matrix
Q  = diag(pvCost(13:18));                % State Weight Matrix
R  = diag(pvCost(19:20));    % Control Weight Matrix
p  = pvCost(21);                        % Prediction Horizon

xCoordDef=pvCost(22);
yCoordDef=pvCost(23);
xdDef=pvCost(24);
ydDef=pvCost(25);
rAvoid=pvCost(26);
rKill=pvCost(27);
barScaleOut=pvCost(28);
barScaleIn=pvCost(29);
barScaleOutFinal=pvCost(30);
barScaleInFinal=pvCost(31);

rRel=sqrt((x(1)-xCoordDef)^2 + (x(2)-yCoordDef)^2);

if stage == p + 1
    % Calculate cost for the terminal stage (the target)
    Ltemp = 0.5*(x - xt).'*Sf*(x - xt);
    if rRel>rKill
        B=-barScaleOutFinal*log(barScaleInFinal*(rRel-rKill));
    else
        B=1e100; %in theory its infinite. For application, its just big
    end
    L=Ltemp+B;

else
    % Calculate cost for intermediate stages
    Ltemp = 0.5*(x - xt).'*Q*(x - xt) + 0.5*u.'*R*u;
    if rRel>rKill
        B=-barScaleOut*log(barScaleIn*(rRel-rKill));
    else
        B=1e100; %in theory its infinite. For application, its just big
    end
    L=Ltemp+B;
end



end



% %Tuning attempt 2
% Q_a=[60;60;0;1;1;0.5]; %[100;100;0;1;1;2]; %[100;100;0;1;1;0]; %state weight matrix
% Sf_a=10*Q_a; %Terminal state weight matrix
% R_a=[0;0]; %Control Weight matrix
% 
% p_a=12; %prediction horizon
% xTarget=[0;0;0;0;0;0];  %reference state=target=origin. This is what attacker is attacking.
% xd=[1;1;0;0]; %placeholder for defender state, which will be changed at each stage
% barrierScaling=[50;5]; %outer and inner scaling constant
% 
% pvcost_a=[xTarget;Sf_a;Q_a;R_a;p_a;xd;avoidRadius; killRadius; barrierScaling];