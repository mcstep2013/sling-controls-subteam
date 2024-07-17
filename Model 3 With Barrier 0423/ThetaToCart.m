function [x,xd,xdd,y,yd,ydd]=ThetaToCart(th1,th1d,th1dd,th2,th2d,th2dd,L1,L2)
    x=L1*cos(th1)+L2*cos(th1+th2);
    y=L1*sin(th1)+L2*sin(th1+th2);
    xd=-L1*sin(th1)*th1d - L2*sin(th1+th2)*(th1d+th2d);
    yd= L1*cos(th1)*th1d + L2*cos(th1+th2)*(th1d+th2d);
    xdd=-L1*cos(th1)*th1d^2 - L1*sin(th1)*th1dd - L2*cos(th1+th2)*(th1d+th2d)^2 - L2*sin(th1+th2)*(th1dd+th2dd);
    ydd=-L1*sin(th1)*th1d^2 + L1*cos(th1)*th1dd - L2*sin(th1+th2)*(th1d+th2d)^2 + L2*cos(th1+th2)*(th1dd+th2dd);
end




