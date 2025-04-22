function x_new = model(x,tau,Dt, Vc)
    mu = 30+20;
    mv = 30+30; 
    mr = 4.14 +0.5;
    mw = 30+70;
    muv = mu-mv;
    Xu = 0.2; 
    Xuu = 25;
    Yv = 55.1;
    Yvv = 101.3;
    Zw = 4.1879;
    Zww = 40.9649;
    gw = 4; % ~400 grams of buoyancy
    if x(3)<0 % surface check
        gw=0;
    end
    Nr = 4.14;
    Nrr = 6.23;
   
    tau_u = tau(1);
    tau_v = tau(2);
    tau_w = tau(3);
    tau_r = tau(4);

    x_new(5) = x(5) + Dt * 1/mu * ( tau_u + mv*x(6)*x(8) - (Xu * x(5) + Xuu *x(5)*abs(x(5))) );
    x_new(6) = x(6) + Dt * 1/mv * ( tau_v + mu*x(5)*x(8) - (Yv * x(6) + Yvv *x(6)*abs(x(6))) );
    x_new(7) = x(7) + Dt * 1/mw * ( tau_w - (Zw * x(7) + Zww *x(7)*abs(x(7))) - gw);
    x_new(8) = x(8) + Dt * 1/mr * ( tau_r + muv*x(5)*x(6) - (Nr * x(8) + Nrr *x(8)*abs(x(8))) );
    x_new(4) = wrapToPi(x(4) + Dt * x_new(8));
    x_new(1) = x(1) + Dt * (x_new(5) * cos(x_new(4)) - x_new(6) * sin(x_new(4)) + Vc(1));
    x_new(2) = x(2) + Dt * (x_new(5) * sin(x_new(4)) + x_new(6) * cos(x_new(4)) + Vc(2));
    x_new(3) = x(3) + Dt * (x_new(7) + Vc(3));

end