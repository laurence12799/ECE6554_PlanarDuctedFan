function [r, dr] = step_ref(t, t0, Ts, A)
    r = A;
    if ( (t-t0) >= Ts )
        dr = 0;
    else
        dr = 1/Ts;
    end
end

