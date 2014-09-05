% Calculate desired th2
t = (t1 - points(1,1)) / (points(end,1) - points(1,1));
t(t>1) = 1;
t(t<0) = 0;
t2_des = 0;
n = size(points, 1) - 1;
for i = 0 : n
    t2_des = t2_des + nchoosek(n,i) * (1-t)^(n-i) * t^i * points(i+1,2);
end
% Calculate desired t2dd
d_phi = n*(t^(n-1)*points(n+1,2) - (1-t)^(n-1)*points(1,2));
for i = 1 : n-1
    d_phi = d_phi + ...
        nchoosek(n,i)*(i-n*t)*t^(i-1)*(1-t)^(n-i-1)*points(i+1,2);
end
d_phi = d_phi / (points(n+1,1)-points(1,1));
dd_phi = (n-1)*n*((1-t)^(n-2)*points(1,2) + t^(n-2)*points(n+1,2)) ...
    + n*((n-1)*(n-2)*t*(1-t)^(n-3) - 2*(n-1)*(1-t)^(n-2))*points(2,2) ...
    + n*((n-1)*(n-2)*t^(n-3)*(1-t) - 2*(n-1)*t^(n-2))*points(n,2);
for i = 2 : n-2
    dd_phi = dd_phi + nchoosek(n,i)*t^(i-2)*(1-t)^(n-i-2)* ...
        (i^2 - i*(2*(n-1)*t + 1) + n*(n-1)*t^2)*points(i+1,2);
end
dd_phi = dd_phi / (points(i+1,1)-points(1,1))^2;
t2dd_des = dd_phi*t1d^2 + d_phi*t1dd;