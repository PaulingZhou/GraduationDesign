clear
N = 447;                                                                                                           %测量数据为447组
dataTempMeasurement = importdata('dataOutputTemp.txt');                     %将测量数据从'dataOutputTemp.txt'中取出
% dataCount = 1:1:N;                                                                                          %横坐标，绘图用
% plot(dataCount,dataTempMeasurement,'-');                                                  %绘出未经过处理的图像
w(1) = 0;                                                                                                           %产生一个1*N的行向量，第一个数为0，
w = randn(1,N);                                                                                                %w为过程噪声（其和后边的v在卡尔曼理论里均为高斯白噪声）
dataTempReal(1) = 25;                                                                                   %实际温度值，赋初值为25℃
a = 1;                                                                                                                 %a为状态转移阵，由于认为温度在这一时刻与下一时刻没有变化，则a=1
for k = 2:N                                                                                                         %系统状态方程,k时刻的状态等于k-1时刻状态乘以状态转移阵加噪声
    dataTempReal(k) = a*dataTempReal(k-1) + w(k-1);                                  %由于未加控制量，故此处忽略
end
V = randn(1,N);                                                                                                  %测量噪声
q1 = std(V);                                                                                                        %测量噪声v的标准差
Rvv = q1 ^ 2;                                                                                                      %测量噪声v的方差
q2 = std(dataTempReal);                                                                                  %温度实际值的标准差
Rxx = q2 ^ 2;                                                                                                       %温度实际值的方差
q3 = std(w);                                                                                                         %过程噪声w的标准差
Rww = q3 ^ 2;                                                                                                      %过程噪声w的方差
c = 1;                                                                                                                    %量测矩阵，由于实测为温度值故c = 1
Y = c*dataTempMeasurement;   
p(1) = 100;
s(1) =27;
for t = 2:N
    p1(t) = a.^2*p(t-1)+Rww;
    b(t) = c*p1(t)/(c.^2*p1(t)+Rvv);
    s(t) = a*s(t-1) + b(t)*(Y(t) - a*c*s(t-1));
    p(t) = p1(t) - c*b(t)*p1(t);
end
t = 1:N;
figure(1);
plot(t,s,'b',t,Y,'g');

