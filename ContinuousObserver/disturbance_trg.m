function Des = disturbance_trg(t_scale,amp)

% Generate triangle-shape disturbance

    n = 3;
    T = 6;
    Des = zeros(n,max(size(t_scale)));
    for i = 1:max(size(t_scale))
    t = t_scale(i);
        if T<t && t<=T+1
            k = t-T;
        elseif T+1<t && t<=T+2
            k = -t+T+2;
        else
            k = 0;
        end
        Des(:,i) = amp*k;
    end

end