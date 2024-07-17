function [Gx, Gmv, Ge]=attackerCostJacFcnV2WBarrier(stage, x, u, e, pvCost)

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
    % Calculate Jacobian matrices for the terminal stage
    dBdx1=-(barScaleOutFinal*(2*xCoordDef - 2*x(1)))/(2*((xCoordDef - x(1))^2 + (yCoordDef - x(2))^2)^(1/2)*(rKill - ((xCoordDef - x(1))^2 + (yCoordDef - x(2))^2)^(1/2)));
    dBdx2=-(barScaleOutFinal*(2*yCoordDef - 2*x(2)))/(2*((xCoordDef - x(1))^2 + (yCoordDef - x(2))^2)^(1/2)*(rKill - ((xCoordDef - x(1))^2 + (yCoordDef - x(2))^2)^(1/2)));
    %The above lines are contributions of the barrier function in cost
    %function. See scraoForCostJacobianAttacker.m for derivation
    Gx  = Sf*(x - xt) + [dBdx1;dBdx2;zeros(4,1)];
    Gmv = zeros(length(u), 1);
else
    % Calculate Jacobian matrices for intermediate stages
    dBdx1=-(barScaleOut*(2*xCoordDef - 2*x(1)))/(2*((xCoordDef - x(1))^2 + (yCoordDef - x(2))^2)^(1/2)*(rKill - ((xCoordDef - x(1))^2 + (yCoordDef - x(2))^2)^(1/2)));
    dBdx2=-(barScaleOut*(2*yCoordDef - 2*x(2)))/(2*((xCoordDef - x(1))^2 + (yCoordDef - x(2))^2)^(1/2)*(rKill - ((xCoordDef - x(1))^2 + (yCoordDef - x(2))^2)^(1/2)));
    %The above lines are contributions of the barrier function in cost
    %function. See scraoForCostJacobianAttacker.m for derivation
    Gx  = Q*(x - xt) + [dBdx1; dBdx2; zeros(4,1)];
    Gmv = R*u;
end
Ge=zeros(length(e),1);

end