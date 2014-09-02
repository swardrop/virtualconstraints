th1 = linspace(0, 2*pi,200);
th2 = linspace(0, -2*pi,200);
l = 2;
p_cm = l/4*(3*[cos(th1); sin(th1)] + [cos(th1+th2); sin(th1+th2)]);
p_cm1 = p_cm - l/4*[cos(th1)+cos(th1+th2); sin(th1)+sin(th1+th2)];
p_cm2 = p_cm + l/4*[cos(th1)+cos(th1+th2); sin(th1)+sin(th1+th2)];

for i = 1 : length(th1)
    hold off
    plot([0, l*cos(th1(i)), l*cos(th1(i))+l*cos(th1(i)+th2(i))], ...
        [0, l*sin(th1(i)), l*sin(th1(i))+l*sin(th1(i)+th2(i))]);
    hold on
    plot(p_cm(1,i), p_cm(2,i), 'ko');
    plot(p_cm1(1,i), p_cm1(2,i), 'bo');
    plot(p_cm2(1,i), p_cm2(2,i), 'ro');
    axis(l*[-2 2 -2 2])
    axis equal
    drawnow
end