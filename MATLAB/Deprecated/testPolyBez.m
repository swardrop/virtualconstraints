function testPolyBez(points)

B = 0;
t = [(0:0.01:1)', (0:0.01:1)'];

for j = 0 : size(points, 1) - 1
    B = B + t.^j .* repmat(findC(points, j), size(t,1), 1);
end
plot(B(:,1), B(:,2));
end

function C = findC(points, j)

n = size(points,1)-1;

A = factorial(n)/factorial(n-j);
sum = 0;

for i = 0 : j
    sum = sum + ((-1)^(i+j) * points(i+1,:)/ ...
        (factorial(i)*(factorial(j-i))));
end

C = A*sum;

end