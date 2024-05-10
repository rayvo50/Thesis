function x_new = model_dyn(x,tau,Dt, Vc)
    mu = 28.5+20;
    mv = 28.5+30; 
    mr = 0.14 +0.5;
    muv = mu-mv;
    du = 0.2; 
    duu = 25;
    dv = 55.1;
    dvv = 0.01;
    dr = 0.14;
    drr = 6.23;
   
    tau_u = tau(1);
    tau_v = 0;
    tau_r = tau(2);

    x_new(4) = x(4) + Dt * 1/mu * ( tau_u + mv * x(5) * x(6) - (du * x(4) + duu *x(4)*abs(x(4))) );
    x_new(5) = x(5) + Dt * 1/mv * ( tau_v + mu * x(4) * x(6) - (dv * x(5) + dvv *x(5)*abs(x(5))) );
    x_new(6) = x(6) + Dt * 1/mr * ( tau_r + muv *x(4) * x(5) - (dr * x(6) + drr *x(6)*abs(x(6))) );
    x_new(3) = wrapToPi( x(3) + Dt * x_new(6) );
    x_new(1) = x(1) + Dt * (x_new(4) * cosd(x_new(3)) - x_new(5) * sind(x_new(3)) + Vc(1));
    x_new(2) = x(2) + Dt * (x_new(4) * sind(x_new(3)) + x_new(5) * cosd(x_new(3)) + Vc(2));

end