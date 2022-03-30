function [r, dr] = ramp_ref(t, t0, m)
    r = m*(t-t0);
    dr = m;
end

