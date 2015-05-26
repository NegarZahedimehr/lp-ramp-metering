function u = explicit_control(n)
% Obtaining the control inputs that explicit control will yield
smallNetwork;
if n(1,1) <= n1_jam - r1_bar
    u(1) = r1_bar;
else
    u(1) = n1_jam - n(1,1);
end

if n(2,1) <= n2_crit - r2_bar
    u(2) = r2_bar;
else if n(2,1) <= n2_crit
        u(2) = n2_crit - n(2,1);
    else
        u(2) = 0;
    end
end

