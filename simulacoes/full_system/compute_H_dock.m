function H = compute_H_dock(x_nav, x_dock)
    
    D = (x_nav(1)-x_dock(1))^2 + (x_nav(2)-x_dock(2))^2;
    H = [   [(x_dock(1)-x_nav(1)), (x_dock(2)-x_nav(2)),0]/sqrt(D);
            [(x_nav(2)-x_dock(2))/D,(x_dock(1)-x_nav(1))/D, 0];
            [-(x_dock(2)-x_nav(2))/D,-(x_nav(1)-x_dock(1))/D, -1] 
        ];
end