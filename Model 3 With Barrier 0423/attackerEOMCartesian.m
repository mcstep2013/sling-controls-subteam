function xdotOut=attackerEOMCartesian(xin,uIn,sysParams)
    [x,y,theta,xdot,ydot,thetadot]=unpack(xin); %cartesian pos/vel and theta (direction of heading)
    %theta is defined counterclockwise positive from x axis, and exists in [0,2pi]
    [tau,f]=unpack(uIn); %tau=torque to change heading, f=force in direction of heading
    [m,I,g]=unpack(sysParams); %mass, moment of inertia, and acceleration due to gravity
    xdotOut=[xdot; ydot; thetadot; (f/m)*cos(theta); (f/m)*sin(theta)-g; tau/I];
end