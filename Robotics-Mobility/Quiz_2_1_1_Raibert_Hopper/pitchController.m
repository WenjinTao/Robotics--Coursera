function Tphi = pitchController(phi,phiDesired,dphi_dt)
kd_phi = 5;
kp_phi = 5;
% Tphi= 0;
e = phiDesired - phi;
d_e = -dphi_dt
Tphi = kd_phi*d_e + kp_phi*e;