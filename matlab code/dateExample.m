x1 = datenum('00:00:00');
x2 = datenum('23:59:59');
x = linspace(x1,x2,24);
plot(x,1:24);
datetick('x',2);