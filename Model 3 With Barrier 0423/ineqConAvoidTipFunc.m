function C = ineqConAvoidTipFunc(stage,x,u,pvcost_a)
%inputs:
    %stage is the stage number from 1 (current control interval) to p+1 (end of horizon)
    %x is the state vector
    %u is the input vector
    %pv is the stage parameter vector

%outputs:
    %C is a scalar that must be nonpositive to be a feasible solution to
    %the NMPC problem

    % Extract target state
    xt = pvcost_a(1:6);
    
    % Extract and Initialize Wiegthing Matrices
    Sf = diag(pvcost_a(7:12));                 % Terminal Weight Matrix
    Q  = diag(pvcost_a(13:18));                % State Weight Matrix
    R  = diag(pvcost_a(19:20));    % Control Weight Matrix
    p  = pvcost_a(21);                        % Prediction Horizon
    
    % Extract position of the defender to avoid
    xDef=pvcost_a(22);
    yDef=pvcost_a(23);
    xDefd=pvcost_a(24);
    yDefd=pvcost_a(25);
    avoidRadius=pvcost_a(26);
    
    %Equation 15B: Real-time game-theoretic model predictive control for differential game of target defense
    %by Mukhtar Sani and others
    C=avoidRadius - sqrt( (x(1)-xDef)^2 + (x(2)-yDef)^2 );

end