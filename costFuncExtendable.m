function L=costFuncExtendable (stage, x, u, pvcost)

    mu=pvcost(1); %nondimensionalized grav parameter
    n=pvcost(2); %dimensional mean motion of earth-moon
    timedimensional=pvcost(3);
    p=pvcost(4); %prediction horizon

    %Extract weight matrices
    Sf = diag(pvcost(5:10));                 % Terminal Weight Matrix
    Q  = diag(pvcost(11:16));                % State Weight Matrix
    R  = diag(pvcost(17:20));    % Control Weight Matrix

    %calculate state of tether tip:
    xtip=getTetherTipState(x,u,pvcost(1:3));
    xrel=x(1:6)-xtip;

    if stage==p+1
        L = 0.5*xrel.'*Sf*xrel;
    else
        L = 0.5*xrel.'*Q*xrel + 0.5*u.'*R*u;
    end

end