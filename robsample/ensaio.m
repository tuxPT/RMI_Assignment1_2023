x = zeros(1,100);
x(3) = 1; 

y = zeros(1,100);

a = 0.8+0.05*sqrt(-1)
b = 0.8-0.05*sqrt(-1)

a = 1.1*a;
b = 1.1*b;

norm(a)

printf("%f y[k-1] - %f y[k-2] + %f x\n",a+b,a*b,(1-(a+b)+a*b))

for i=3:98
  %y(i) = (-0.5*y(i-2)-0.25*y(i-1)+x(i)/0.57413);
  y(i) = (a+b)*y(i-1) - a*b*y(i-2) + (1-(a+b)+a*b)*x(i);
endfor

x = ones(1,100);
x(1:2) = [0 0]; 

y1 = zeros(1,100);

for i=3:98
  %y(i) = (-0.5*y(i-2)-0.25*y(i-1)+x(i)/0.57413);
  y1(i) = (a+b)*y1(i-1) - a*b*y1(i-2) + (1-(a+b)+a*b)*x(i);
endfor

figure
plot(y)
figure
plot(y1)