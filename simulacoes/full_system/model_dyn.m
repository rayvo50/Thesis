function x_new = model_dyn(x,u,Dt, Vc)
    mu = 1; mv = 1; mr = 1; du = 0.1; dv = 0.1; dr = 0.1;

    tau_u = u(1);
    tau_r = u(2);

    x_new(4) = x(4) + Dt * 1/mu * (tau_u + mv * x(5) * x(6) - (du * x(4) + duu *x(4)*abs(x(4))) );
    x_new(5) = x(5) + Dt * 1/mv * (  0   + mu * x(4) * x(6) - (dv * x(5) + dvv *x(5)*abs(x(5))) );
    x_new(6) = x(6) + Dt * 1/mr * (tau_r + muv *x(4) * x(5) - (dr * x(6) + drr *x(6)*abs(x(6))) );
    x_new(3) = x(3) + Dt * x_new(6);
    x_new(1) = x(1) + Dt * (x_new(4) * cosd(x_new(3)) - x_new(5) * sind(x_new(3)) + Vc(1));
    x_new(2) = x(2) + Dt * (x_new(4) * sind(x_new(3)) + x_new(5) * cosd(x_new(3)) + Vc(2));

end