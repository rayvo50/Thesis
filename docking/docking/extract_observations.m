function observations = extract_observations(usbl_auv, usbl_dock)
    % Convert RBE (Range, Bearing, Elevation) to XYZ coordinates
    xyz_auv = rbe2xyz(usbl_auv);
    xyz_dock = rbe2xyz(usbl_dock);
    
    % Compute relative yaw
    r1 = -dot(xyz_dock, xyz_auv);
    r2 = cross(xyz_dock, xyz_auv);
    yaw = atan2(r2(3), r1);
    
    % Return observations
    observations = [xyz_dock(1); xyz_dock(2); xyz_dock(3); yaw];
end
