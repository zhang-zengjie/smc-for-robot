function Des = disturbance_sin(t_scale,amp)

% Generate sinusoidal disturbance

    n = 3;
    T = 6;
    Des = zeros(n,max(size(t_scale)));
    for i = 1:max(size(t_scale))
        if T<t_scale(i) && t_scale(i)<T+2
            k = sin(pi/2*(t_scale(i)-T));
        else
            k = 0;
        end
        Des(:,i) = amp*k;
    end

end