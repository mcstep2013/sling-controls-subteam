function L = attackerCostFcnV2(stage, x, u, e, pvcost)
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

% Extract Desired State
xt = pvcost(1:6);

% Extract and Initialize Wiegthing Matrices
Sf = diag(pvcost(7:12));                 % Terminal Weight Matrix
Q  = diag(pvcost(13:18));                % State Weight Matrix
R  = diag(pvcost(19:20));    % Control Weight Matrix
p  = pvcost(21);                        % Prediction Horizon

if stage == p + 1
    % Calculate cost for the terminal stage (the target)
    L = 0.5*(x - xt).'*Sf*(x - xt);
else
    % Calculate cost for intermediate stages
    L = 0.5*(x - xt).'*Q*(x - xt) + 0.5*u.'*R*u;
end
end