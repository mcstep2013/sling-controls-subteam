function L = attackerCostFcnSimplified(stage, x, u, pvcost_a)
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
xt = pvcost_a(1:2);

% Extract and Initialize Wiegthing Matrices
Sf = diag(pvcost_a(3:4));                 % Terminal Weight Matrix
Q  = diag(pvcost_a(5:6));                % State Weight Matrix
R  = diag(pvcost_a(7:8));    % Control Weight Matrix
p  = pvcost_a(9);                        % Prediction Horizon

if stage == p + 1
    % Calculate cost for the terminal stage (the target)
    L = 0.5*(x - xt).'*Sf*(x - xt);
else
    % Calculate cost for intermediate stages
    L = 0.5*(x - xt).'*Q*(x - xt) + 0.5*u.'*R*u;
end
end