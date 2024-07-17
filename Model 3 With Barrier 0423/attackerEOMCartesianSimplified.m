function xdotOut=attackerEOMCartesianSimplified(xin,uIn,sysParams)
    [x,y]=unpack(xin); %cartesian pos/vel and theta (direction of heading)
    %theta is defined counterclockwise positive from x axis, and exists in [0,2pi]
    [theta,v]=unpack(uIn); %tau=torque to change heading, f=force in direction of heading
    [m,I,g]=unpack(sysParams); %mass, moment of inertia, and acceleration due to gravity
    xdotOut=[v*cos(theta); v*sin(theta)];
end