function u = lookup_u(tu, t)
i = find(tu.t <= t, 1, 'last'); %trova l'ultimo indice in cui tu.t<=t
[m,n] = size(tu.u);
if (m==1) || (n==1)
    if i<length(tu.t)
        u = (tu.u(i)*(tu.t(i+1) -t) + tu.u(i+1)*(t-tu.t(i)))/(tu.t(i+1)-tu.t(i));
    else
        u = tu.u(i); 
    end
else
    if i < length(tu.t)
        u = (tu.u(:,i)*(tu.t(i+1) -t) + ...
            tu.u(:,i+1)*(t-tu.t(i)))/(tu.t(i+1)-tu.t(i));
    else
        u = tu.u(:,i);
    end
end
end