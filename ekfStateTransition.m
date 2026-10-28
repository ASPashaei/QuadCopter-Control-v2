function xk1 = ekfStateTransition(xk, u)
% xk = [px py pz vx vy vz phi theta psi]
% u  = [ax ay az p q r] 
Ts = 0.005;                       
phi = xk(7); theta = xk(8); psi = xk(9);
a_b = u(1:3);  w_b = u(4:6);

cph=cos(phi); sph=sin(phi);
cth=cos(theta); sth=sin(theta); cps=cos(psi); sps=sin(psi);

R = [ cth*cps, sph*sth*cps-cph*sps, cph*sth*cps+sph*sps;
      cth*sps, sph*sth*sps+cph*cps, cph*sth*sps-sph*cps;
      -sth,    sph*cth,             cph*cth ];        % body -> nav (ENU), ZYX
g = [0; 0; -9.81];
T = [ 1, sph*tan(theta), cph*tan(theta);
      0, cph,           -sph;
      0, sph/cth,        cph/cth ];                   % body rates -> Euler rates

xdot = [ xk(4:6); R*a_b + g; T*w_b ];
xk1  = xk + Ts*xdot;                                  % forward Euler
end


