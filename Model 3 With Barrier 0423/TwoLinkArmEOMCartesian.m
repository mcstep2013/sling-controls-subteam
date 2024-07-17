function xdotout=TwoLinkArmEOMCartesian(xin,tauIn,sysParams)
[x,y,xdot,ydot]=unpack(xin);
[L1,CM1,M1,J1,L2,CM2,M2,J2,g]=unpack(sysParams);

%% From https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
%(Using the positive theta2 arbitrarily)... SHOULD I PUT MORE THOUGHT INTO
%THAT?
th2Temp=acos((x^2 + y^2 -L1^2 -L2^2)/(2*L1*L2));
if ~isreal(th2Temp)
    pausehere=0;
end
th1Temp=atan2(y,x)-atan2((L2*sin(th2Temp)),(L1+L2*cos(th2Temp)));
%th1Temp is defined counterclockwise positive wrt positive x axis
%th2Temp is defined counterclockwise positive wrt the first link

%% Debugging test
% xTestPrelim=L1*cos(th1Temp)+L2*cos(th1Temp+th2Temp);
% yTestPrelim=L1*sin(th1Temp)+L2*sin(th1Temp+th2Temp);

%% From scrapForAnalyticSolns.m
th2TempDot=-(2*xdot*x + 2*ydot*y)/(2*L1*L2*(1 - (- L1^2 - L2^2 + x^2 + y^2)^2/(4*L1^2*L2^2))^(1/2));
th1TempDot=(xdot*x^3 + ydot*y^3 + xdot*x*y^2 + ydot*x^2*y - L1^2*xdot*x + L2^2*xdot*x - L1^2*ydot*y + L2^2*ydot*y - 2*L1*L2*xdot*y*(1 - (- L1^2 - L2^2 + x^2 + y^2)^2/(4*L1^2*L2^2))^(1/2) + 2*L1*L2*ydot*x*(1 - (- L1^2 - L2^2 + x^2 + y^2)^2/(4*L1^2*L2^2))^(1/2))/(2*L1*L2*(1 - (- L1^2 - L2^2 + x^2 + y^2)^2/(4*L1^2*L2^2))^(1/2)*(x^2 + y^2));

%% Redefining variables to fit with paper below
th1=pi/2-th1Temp;
th2=-th2Temp;
th1dot=-th1TempDot;
th2dot=-th2TempDot;
%th1 is defined clockwise positive wrt y axis
%th2 is defined clockwise positive wrt the first link

%% From Model predictive control of a two-link robot arm by El-Hadi Guechi; Samir Bouzoualegh; Lotfi Messikh; Sašo Blažic
G=[-(M1+M2)*g*L1*sin(th1)-M2*g*L2*sin(th1+th2);
   -M2*g*L2*sin(th1+th2)];
C=[-M2*L1*L2*(2*th1dot*th2dot+th1dot^2)*sin(th2);
    -M2*L1*L2*th1dot*th2dot*sin(th2)];
D1=(M1+M2)*L1^2 + M2*L2^2 + 2*M2*L1*L2*cos(th2);
D2=M2*L2^2 + M2*L1*L2*cos(th2);
D3=D2;
D4=M2*L2^2;
M=[D1, D2; D3, D4];
Minv=M^-1;
thddot=Minv*(tauIn-C-G);
th1ddot=thddot(1);
th2ddot=thddot(2);

%% From derivatives of eqation 1 in the above paper
xTest=L1*sin(th1)+L2*sin(th1+th2); %should be the same as the x we passed in
yTest=L1*cos(th1)+L2*cos(th1+th2);
xdotTest=L1*cos(th1)*th1dot + L2*cos(th1+th2)*(th1dot+th2dot); %should be the same as the xdot we passed in
ydotTest=-L1*sin(th1)*th1dot - L2*sin(th1+th2)*(th1dot+th2dot); %should be the same as the ydot we passed in
xdd=-L1*sin(th1)*th1dot^2 + L1*cos(th1)*th1ddot-L2*sin(th1+th2)*(th1dot+th2dot)^2 + L2*cos(th1+th2)*(th1ddot+th2ddot);
ydd=-L1*cos(th1)*th1dot^2 - L1*sin(th1)*th1ddot-L2*cos(th1+th2)*(th1dot+th2dot)^2 - L2*sin(th1+th2)*(th1ddot+th2ddot);
xdotout=[xdotTest;ydotTest;xdd;ydd];


end