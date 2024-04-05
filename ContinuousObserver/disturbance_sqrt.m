function Des = disturbance_sqrt(t_scale,amp)

% Generate square-shape disturbance

n = 3;
T = 6;
Des = zeros(n,max(size(t_scale)));

    for i = 1:max(size(t_scale))
        if T<t_scale(i) && t_scale(i)<=T+1
            k = 1;
        else
            k = 0;
        end
        Des(:,i) = amp*k;
    end

end