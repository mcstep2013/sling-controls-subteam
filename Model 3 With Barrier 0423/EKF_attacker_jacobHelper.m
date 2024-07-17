function [A] = EKF_jaco_help(xk, uk)

% Retrieve physical parameters from pvstate
l1 = 0.45; %pvstate(1);    % Length of link 1 (m)
s1 = l1/2; %pvstate(2);    % Distance from joint 1 to center of mass of link 1 (m)
m1 = 0.2; %pvstate(3);    % Mass of link 1 (kg)
J1 = (1/3)*m1*l1^2; %pvstate(4);    % Moment of inertia of link 1 (kg*m^2)
l2 = 0.35; %pvstate(5);    % Length of link 2 (m)
s2 = l2/2; %pvstate(6);    % Distance from joint 2 to center of mass of link 2 (m)
m2 = 0.6; %pvstate(7);    % Mass of link 2 (kg)
J2 = (1/3)*m2*l2^2; %pvstate(8);    % Moment of inertia of link 2 (kg*m^2)
g  = 9.81; %pvstate(9);    % Acceleration due to gravity (m/s^2)


% Control Input
tau1 = uk(1,:);
tau2 = uk(2,:);

% States
x1 = xk(1,:);
x2 = xk(2,:);
x3 = xk(3,:);
x4 = xk(4,:);

t2 = cos(x1);
t3 = cos(x2);
t4 = sin(x1);
t5 = sin(x2);
t6 = J1.*J2;
t7 = J2.*tau1;
t8 = J2.*tau2;
t9 = x1+x2;
t10 = l1.^2;
t11 = l1.^3;
t12 = s1.^2;
t13 = s2.^2;
t14 = s2.^3;
t15 = m2.^2;
t16 = x3.^2;
t17 = x4.^2;
t18 = t3.^2;
t19 = t5.^2;
t20 = cos(t9);
t21 = sin(t9);
t22 = -t8;
t23 = m2.*t13;
t24 = m1.*J2.*t12;
t25 = m2.*J2.*t10;
t29 = l1.*s2.*m2.*t3;
t30 = s1.*m1.*J2.*g.*t4;
t31 = l1.*m2.*J2.*g.*t4;
t32 = s1.*m1.*J2.*g.*t2;
t33 = l1.*m2.*J2.*g.*t2;
t35 = l1.*s2.*m2.*J2.*t5.*x3.*2.0;
t36 = l1.*s2.*m2.*J2.*t5.*x4.*2.0;
t46 = t10.*t13.*t15;
t50 = l1.*s2.*m2.*J2.*t5.*t16;
t51 = l1.*s2.*m2.*J2.*t5.*t17;
t54 = l1.*g.*t2.*t13.*t15;
t55 = l1.*g.*t4.*t13.*t15;
t63 = l1.*t5.*t14.*t15.*x3.*2.0;
t64 = l1.*t5.*t14.*t15.*x4.*2.0;
t65 = l1.*t3.*t14.*t15.*t16;
t66 = l1.*t3.*t14.*t15.*t17;
t67 = l1.*t5.*t14.*t15.*t16;
t68 = l1.*t5.*t14.*t15.*t17;
t69 = l1.*t3.*t14.*t15.*x3.*x4.*2.0;
t26 = J1.*t23;
t27 = t23.*tau1;
t28 = t23.*tau2;
t37 = l2.*m2.*J2.*g.*t20;
t38 = s2.*m2.*J2.*g.*t20;
t39 = m1.*t12.*t23;
t40 = J1.*s2.*m2.*g.*t21;
t41 = l2.*m2.*J2.*g.*t21;
t42 = s2.*m2.*J2.*g.*t21;
t43 = -t32;
t44 = -t33;
t45 = s1.*m1.*g.*t2.*t23;
t47 = J2.*t16.*t29;
t48 = J2.*t17.*t29;
t49 = s1.*m1.*g.*t4.*t23;
t52 = J2.*t29.*x3.*x4.*2.0;
t53 = t35.*x4;
t59 = g.*t14.*t15.*t20;
t60 = g.*t14.*t15.*t21;
t61 = m1.*s2.*m2.*g.*t12.*t21;
t70 = t63.*x4;
t71 = l2.*g.*t13.*t15.*t20;
t72 = s2.*g.*t10.*t15.*t21;
t73 = l2.*g.*t13.*t15.*t21;
t74 = -t54;
t75 = J2+t23+t29;
t76 = t18.*t46;
t78 = l1.*l2.*s2.*g.*t3.*t15.*t21;
t80 = l1.*g.*t3.*t13.*t15.*t21;
t84 = t3.*t5.*t46.*x3.*2.0;
t85 = t3.*t5.*t46.*x4.*2.0;
t34 = -t28;
t56 = -t37;
t57 = -t40;
t58 = -t42;
t62 = -t45;
t77 = -t60;
t79 = -t61;
t81 = -t71;
t82 = -t72;
t83 = -t76;
t86 = t80.*2.0;
t87 = -t80;
t88 = -t86;
t89 = t6+t24+t25+t26+t39+t46+t83;
t90 = 1.0./t89;
t91 = t90.^2;
et1 = -t90.*(t41+t47+t48+t52+t57+t58+t65+t66+t69+t73+t77+t78+t79+t82+t88+t16.*t76.*2.0+t17.*t76+J1.*t16.*t29-t16.*t19.*t46.*2.0-t17.*t19.*t46+t76.*x3.*x4.*2.0+m1.*t12.*t16.*t29-t19.*t46.*x3.*x4.*2.0-l1.*s2.*m2.*t5.*tau1+l1.*s2.*m2.*t5.*tau2.*2.0+s2.*t3.*t11.*t15.*t16+s2.*g.*t2.*t5.*t10.*t15-l1.*g.*t5.*t13.*t15.*t20.*2.0+l1.*l2.*s2.*g.*t5.*t15.*t20+l1.*s1.*m1.*s2.*m2.*g.*t2.*t5);
et2 = t3.*t5.*t46.*t91.*(t7+t22+t27+t34+t38+t43+t44+t50+t51+t53+t56+t59+t62+t67+t68+t70+t74+t81-J1.*tau2+t29.*tau1-t29.*tau2.*2.0+t84.*x4-m1.*t12.*tau2-m2.*t10.*tau2+t3.*t5.*t16.*t46.*2.0+t3.*t5.*t17.*t46+J1.*s2.*m2.*g.*t20-s1.*m1.*g.*t2.*t29+s2.*g.*t10.*t15.*t20+s2.*t5.*t11.*t15.*t16+l1.*J1.*s2.*m2.*t5.*t16+m1.*s2.*m2.*g.*t12.*t20-s2.*g.*t2.*t3.*t10.*t15+l1.*g.*t3.*t13.*t15.*t20.*2.0+l1.*m1.*s2.*m2.*t5.*t12.*t16-l1.*l2.*s2.*g.*t3.*t15.*t20).*2.0;
mt1 = [0.0,0.0,t90.*(t30+t31+t41+t49+t55+t58+t73+t77+t87),-t90.*(t30+t31+t41+t49+t55+t57+t58+t73+t77+t78+t79+t82+t88+s1.*m1.*g.*t4.*t29+s2.*g.*t3.*t4.*t10.*t15),0.0,0.0,t90.*(t41+t47+t48+t52+t58+t65+t66+t69+t73+t77+t87+t16.*t76-t16.*t19.*t46+l1.*s2.*m2.*t5.*tau2-l1.*g.*t5.*t13.*t15.*t20)-t3.*t5.*t46.*t91.*(t7+t22+t27+t34+t38+t43+t44+t50+t51+t53+t56+t59+t62+t67+t68+t70+t74+t81-t29.*tau2+t3.*t5.*t16.*t46+l1.*g.*t3.*t13.*t15.*t20).*2.0,et1+et2,1.0,0.0,t90.*(t35+t36+t63+t64+t84),-t90.*(t35+t36+t63+t64+t85+t3.*t5.*t46.*x3.*4.0+s2.*t5.*t11.*t15.*x3.*2.0+l1.*J1.*s2.*m2.*t5.*x3.*2.0+l1.*m1.*s2.*m2.*t5.*t12.*x3.*2.0),0.0,1.0,t90.*(t35+t36+t63+t64)];
mt2 = -t90.*(t35+t36+t63+t64+t84+t85);
A = reshape([mt1,mt2],4,4);
if nargout > 1
    t92 = t75.*t90;
    t93 = -t92;
    B = reshape([0.0,0.0,t90.*(J2+t23),t93,0.0,0.0,t93,t90.*(J1+t29+t75+m1.*t12+m2.*t10)],[4,2]);
end


end