function C = ineqConAvoidTipFuncSimplified(stage,x,u,pvcost_a)
%inputs:
    %stage is the stage number from 1 (current control interval) to p+1 (end of horizon)
    %x is the state vector
    %u is the input vector
    %pv is the stage parameter vector

%outputs:
    %C is a scalar that must be nonpositive to be a feasible solution to
    %the NMPC problem

    % Extract target state
    xt = pvcost_a(1:2);
    
    % Extract and Initialize Wiegthing Matrices
    Sf = diag(pvcost_a(3:4));                 % Terminal Weight Matrix
    Q  = diag(pvcost_a(5:6));                % State Weight Matrix
    R  = diag(pvcost_a(7:8));    % Control Weight Matrix
    p  = pvcost_a(9);                        % Prediction Horizon
    
    % Extract position of the defender to avoid
    xDef=pvcost_a(10);
    yDef=pvcost_a(11);
    avoidRadius=pvcost_a(12);

    %Equation 15B: Real-time game-theoretic model predictive control for differential game of target defense
    %by Mukhtar Sani and others
    C=avoidRadius - sqrt( (x(1)-xDef)^2 + (x(2)-yDef)^2 );

end