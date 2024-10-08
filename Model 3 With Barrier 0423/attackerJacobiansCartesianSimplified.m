function [dfdx,dfdu] = attackerJacobiansCartesianSimplified(in1,in2,in3)
%attackerJacobiansCartesianSimplified
%    [DFDX,DFDU] = attackerJacobiansCartesianSimplified(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 24.1.
%    22-Apr-2024 13:17:14

uIn1 = in2(1,:);
uIn2 = in2(2,:);
dfdx = reshape([0.0,0.0,0.0,0.0],[2,2]);
if nargout > 1
    t2 = cos(uIn1);
    t3 = sin(uIn1);
    dfdu = reshape([-t3.*uIn2,t2.*uIn2,t2,t3],[2,2]);
end
end
