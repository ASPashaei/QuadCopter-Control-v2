function dX=quad_dynamics(t,X,U,params)
        m = params.m;
        g= params.g;
        Ixx=params.Ixx;Iyy=params.Iyy;Izz=params.Izz;
        x=X(1);y=X(2);z=X(3);
        vx=X(4);vy=X(5);vz=X(6);
        phi=X(7);theta=X(8);psi=X(9);
        p = X(10);q=X(11);r=X(12);
        U1=U(1);U2=U(2);U3=U(3);U4=U(4);

        % Rotational Dyanamics:
        p_dot = (U2-(Izz-Iyy)*q*r)/Ixx;
        q_dot = (U3-(Ixx-Izz)*p*r)/Iyy;
        r_dot = (U4-(Iyy-Ixx)*p*q)/Izz;

        % 
        phi_dot= p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
        theta_dot = q*cos(phi)-r*sin(phi);
        psi_dot = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);

        % Translational
        ax = (U1/m)*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
        ay = (U1/m)*(cos(phi)*sin(theta)*sin(psi)- sin(phi)*cos(psi));
        az = (-1*g) + (U1/m)*(cos(phi)*cos(theta));

        dX = [X(4);X(5);X(6);ax;ay;az;phi_dot;theta_dot;psi_dot;p_dot;q_dot;r_dot];
end 


