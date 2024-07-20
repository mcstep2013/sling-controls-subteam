function xtip=getTetherTipState(x,u,params)

    %Tether orientation and length states
    l=x(13); %length of tether
    alpha=x(14); %nagle of tether wrt inertial
    alphad=x(15); %rate of change of that
    ld=u(4); %rate of change of length of tether

    %pvcost
    mu=params(1); %nondimensionalized grav parameter
    n=params(2); %dimensional mean motion of earth-moon
    timedimensional=params(3);

    %Calculating state of tether tip
    % xtipB=[l;0;0;ld;0;0];%position of tether tip in B frame (rotational, fixed to tether COM, that's x,y,z,xdot,ydot,zdot)
    beta=n*timedimensional - alpha; %angle btw tether and synodic frame
    xtiprelCM=[l*cos(beta); -l*sin(beta); 0; (alphad-n)*l*sin(beta)+ld*cos(beta); (alphad-n)*l*cos(beta)-ld*sin(beta);0];
    xtip=xtiprelCM + x(7:12);

end