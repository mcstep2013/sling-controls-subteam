function L = twolinkCostFcn(stage, x, u, pvcost)
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
xf = pvcost(1:4);

% Extract and Initialize Wiegthing Matrices
Sf = diag(pvcost(5:8));                 % Terminal Weight Matrix
Q  = diag(pvcost(9:12));                % State Weight Matrix
R  = diag(pvcost(12+(1:length(u))));    % Control Weight Matrix
p  = pvcost(15);                        % Prediction Horizon

if stage == p + 1
    % Calculate cost for the terminal stage
    L = 0.5*(x - xf).'*Sf*(x - xf);
else
    % Calculate cost for intermediate stages
    L = 0.5*(x - xf).'*Q*(x - xf) + 0.5*u.'*R*u;
end
end