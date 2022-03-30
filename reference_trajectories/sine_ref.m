function [r, dr] = sine_ref(t, A, omega, phase)
    r = A*sin(omega*t + phase);
    dr = omega*A*cos(omega*t + phase);
end

