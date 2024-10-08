function [dLdx,dLdu] = ExtendableTetherCostJacobiansPT2(in1,in2,in3)
%ExtendableTetherCostJacobiansPT2
%    [dLdx,dLdu] = ExtendableTetherCostJacobiansPT2(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 24.1.
%    19-Jul-2024 19:02:59

pvcost2 = in3(2,:);
pvcost3 = in3(3,:);
pvcost11 = in3(11,:);
pvcost12 = in3(12,:);
pvcost13 = in3(13,:);
pvcost14 = in3(14,:);
pvcost15 = in3(15,:);
pvcost16 = in3(16,:);
pvcost17 = in3(17,:);
pvcost18 = in3(18,:);
pvcost19 = in3(19,:);
pvcost20 = in3(20,:);
u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
x1 = in1(1,:);
x2 = in1(2,:);
x3 = in1(3,:);
x4 = in1(4,:);
x5 = in1(5,:);
x6 = in1(6,:);
x7 = in1(7,:);
x8 = in1(8,:);
x9 = in1(9,:);
x10 = in1(10,:);
x11 = in1(11,:);
x12 = in1(12,:);
x13 = in1(13,:);
x14 = in1(14,:);
x15 = in1(15,:);
t2 = pvcost2.*pvcost3;
t3 = -x1;
t4 = -x2;
t5 = -x4;
t6 = -x9;
t7 = -x11;
t8 = -x12;
t9 = -x15;
t11 = x1./2.0;
t12 = x2./2.0;
t13 = x3./2.0;
t14 = x4./2.0;
t15 = x5./2.0;
t16 = x6./2.0;
t17 = x7./2.0;
t18 = x8./2.0;
t19 = x9./2.0;
t20 = x10./2.0;
t21 = x11./2.0;
t22 = x12./2.0;
t10 = -t2;
t23 = pvcost2+t9;
t24 = t6+x3;
t25 = t8+x6;
t26 = -t11;
t27 = -t12;
t28 = -t14;
t29 = -t19;
t30 = -t21;
t31 = -t22;
t32 = t10+x14;
t39 = (pvcost13.*t24)./2.0;
t40 = (pvcost16.*t25)./2.0;
t41 = t13+t29;
t42 = t16+t31;
t33 = cos(t32);
t34 = sin(t32);
t49 = pvcost13.*t41;
t50 = pvcost16.*t42;
t35 = t33.*u4;
t36 = t33.*x13;
t37 = t34.*u4;
t38 = t34.*x13;
t43 = -t37;
t44 = t35./2.0;
t45 = t36./2.0;
t46 = t37./2.0;
t47 = t38./2.0;
t51 = t3+t36+x7;
t52 = t23.*t38;
t53 = t4+t38+x8;
t54 = t23.*t36;
t48 = -t46;
t55 = t23.*t47;
t56 = t23.*t45;
t57 = (pvcost11.*t51)./2.0;
t58 = (pvcost12.*t53)./2.0;
t59 = t17+t26+t45;
t60 = t18+t27+t47;
t63 = t5+t35+t52+x10;
t64 = t7+t43+t54+x5;
t61 = pvcost11.*t59;
t62 = pvcost12.*t60;
t65 = (pvcost14.*t63)./2.0;
t66 = (pvcost15.*t64)./2.0;
t67 = t20+t28+t44+t55;
t68 = t15+t30+t48+t56;
t69 = pvcost14.*t67;
t70 = pvcost15.*t68;
dLdx = [-t57-t61,-t58-t62,t39+t49,-t65-t69,t66+t70,t40+t50,t57+t61,t58+t62,-t39-t49,t65+t69,-t66-t70,-t40-t50,t33.*t57+t34.*t58+t33.*t61+t34.*t62+t23.*t33.*t66+t23.*t34.*t65+t23.*t33.*t70+t23.*t34.*t69,-t70.*(t35+t52)+t36.*t62-t38.*t61-t69.*(t37-t54)-pvcost15.*t64.*(t44+t55)-(pvcost11.*t38.*t51)./2.0+pvcost12.*t45.*t53-pvcost14.*t63.*(t46-t54./2.0),-t36.*t70-t38.*t69-(pvcost14.*t38.*t63)./2.0-(pvcost15.*t36.*t64)./2.0];
if nargout > 1
    dLdu = [pvcost17.*u1,pvcost18.*u2,pvcost19.*u3,pvcost20.*u4+t33.*t65+t33.*t69-t34.*t70-(pvcost15.*t34.*t64)./2.0];
end
end
