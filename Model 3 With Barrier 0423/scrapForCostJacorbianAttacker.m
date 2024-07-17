clear
clc
close all

syms pvCost [31, 1] real
syms x [6,1] real
syms u [2,1] real

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

Ltemp = 0.5*(x - xt).'*Q*(x - xt) + 0.5*u.'*R*u;
B=-barScaleOut*log(barScaleIn*(rRel-rKill));
L=Ltemp+B;
dBdx=jacobian(B,x);
dBdu=jacobian(B,u);

% % % if stage == p + 1
% % %     % Calculate cost for the terminal stage (the target)
% % %     Ltemp = 0.5*(x - xt).'*Sf*(x - xt);
% % %     if rRel>rKill
% % %         B=-barScaleOutFinal*log(barScaleInFinal*(rRel-rKill));
% % %     else
% % %         B=inf;
% % %     end
% % %     L=Ltemp+B;
% % % 
% % % else
% % %     % Calculate cost for intermediate stages
% % %     Ltemp = 0.5*(x - xt).'*Q*(x - xt) + 0.5*u.'*R*u;
% % %     if rRel>rKill
% % %         B=-barScaleOut*log(barScaleIn*(rRel-rKill));
% % %     else
% % %         B=inf;
% % %     end
% % %     L=Ltemp+B;
% % % end



