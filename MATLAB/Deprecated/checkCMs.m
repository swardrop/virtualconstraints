th1 = linspace(-pi, pi,1000);
th2 = linspace(-pi/2, pi/2,1000);
l = 2;
p_cm = l/4*(3*[sin(th2); cos(th2)] + [sin(th1-th2); -cos(th1-th2)]);
p_cm1 = p_cm - l/4*[sin(th2)+sin(th1-th2); cos(th2)-cos(th1-th2)];
p_cm2 = p_cm + l/4*[sin(th2)+sin(th1-th2); cos(th2)-cos(th1-th2)];

for i = 1 : length(th1)
    hold off
    plot([0, l*sin(th2(i)), l*sin(th2(i))+l*sin(th1(i)-th2(i))], ...
        [0, l*cos(th2(i)), l*cos(th2(i))-l*cos(th1(i)-th2(i))]);
    hold on
    plot(p_cm(1,i), p_cm(2,i), 'ko');
    plot(p_cm1(1,i), p_cm1(2,i), 'bo');
    plot(p_cm2(1,i), p_cm2(2,i), 'ro');
    axis(l*[-2 2 -2 2])
    axis equal
    drawnow
end