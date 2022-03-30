function [torque, phi] = static_SFB(x, r, dr)
%% Calculates linear state-feedback control input:
%       u(t) = Kr*[r(t); dr(t)] - Kx'*x(t)
    Kx = [0; 0; 0; 0]; Kr = [1; 0];
    torque = Kr'*[r; dr] - Kx'*x;
    phi = 0;
end

