function [r, dr] = custom_periodic_ref_1(t, t0, T)
%% Piecewise continuous
    tRem = rem(t, T);
    A = 1; 
    if ( tRem >= 0 ) % Step
        [r, dr] = step_ref(tRem, t0, 0.0001, A);
    elseif ( tRem >= (1/5)*T ) % Up ramp + step
        [temp_r1, temp_dr1] = step_ref(tRem, t0, 0.0001, A);
        [temp_r2, temp_dr2] = ramp_ref(tRem, t0, 1);
        r = temp_r1 + temp_r2; dr = temp_dr1 + temp_dr2;
    elseif ( tRem >= (2/5)*T ) % Step
        [r, dr] = step_ref(tRem, t0, 0.0001, A+(1/5)*T);
    elseif ( tRem >= (3/5)*T ) % Down ramp + step;
        [temp_r1, temp_dr1] = step_ref(tRem, t0, 0.0001, A+(1/5)*T);
        [temp_r2, temp_dr2] = ramp_ref(tRem, t0, -2);
        r = temp_r1 + temp_r2; dr = temp_dr1 + temp_dr2;
    elseif ( tRem >= (4/5)*T ) % Step 
        [r, dr] = step_ref(tRem, t0, 0.0001, A-(1/5)*T);
    end
    
end

