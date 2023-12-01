function [vx, omega] = sendVel(LinearV,dir,pose,lim_omega)

    vx= LinearV;
    head_to_go = atan2(dir(:,2),dir(:,1));
    omega = wrapToPi(head_to_go(1,1) - pose(1,3));
    if omega < -lim_omega
        omega = -lim_omega;
    elseif omega > lim_omega
        omega = lim_omega;
    end
end