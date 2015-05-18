function[s,N] = KalmanFilteringTemp(dataTempMeasurement)
N = length(dataTempMeasurement);                                                                                                           %测量数据为447组
% dataTempMeasurement = importdata('dataOutputTemp.txt');                     %将测量数据从'dataOutputTemp.txt'中取出
% dataCount = 1:1:N;                                                                                      %横坐标，绘图用
% plot(dataCount,dataTempMeasurement,'-');                                              %绘出未经过处理的图像
w = 0.025*randn(1,N);                                                                                       %w为过程噪声（其和后边的v在卡尔曼理论里均为高斯白噪声）
w(1) = 0;                                                                                                           %产生一个1*N的行向量，第一个数为0，
dataTempReal(1) = dataTempMeasurement(1);                                           %实际温度值，赋初值为测量的第一个值
a = 1;                                                                                                                 %a为状态转移阵，由于认为温度在这一时刻与下一时刻没有变化，则a=1
for k = 2:N                                                                                                         %系统状态方程,k时刻的状态等于k-1时刻状态乘以状态转移阵加噪声
    dataTempReal(k) = a*dataTempReal(k-1) + w(k-1);                                  %由于未加控制量，故此处忽略控制量U(k)
end
V = 0.15*randn(1,N);                                                                                         %测量噪声
q1 = std(V);                                                                                                        %测量噪声v的标准差
Rvv = q1 ^ 2;                                                                                                      %测量噪声v的方差
q2 = std(dataTempReal);                                                                                  %温度实际值的标准差
Rxx = q2 ^ 2;                                                                                                       %温度实际值的方差
q3 = std(w);                                                                                                         %过程噪声w的标准差
Rww = q3 ^ 2;                                                                                                      %过程噪声w的方差
c = 1;                                                                                                                    %量测矩阵，由于实测为温度值故c = 1
Y = c*dataTempMeasurement;                                                                          %量测方差
p(1) = 0;                                                                                                               %初始最优化估计协方差                                                                                                        
s(1) =dataTempMeasurement(1);                                                                      %s(1)表示为初始最优化估计，初始时没有估计，则取测量值
for t = 2:N
    p1(t) = a.^2*p(t-1)+Rww;                                                                                  %p1为一步估计得协方差，此式从t-1时刻最优化估计s的协方差得到t-1时刻到t时刻一步估计协方差
    b(t) = c*p1(t)/(c.^2*p1(t)+Rvv);                                                                         %b为卡尔曼增益，其意义表示为状态误差的协方差与量测误差的协方差之比
    s(t) = a*s(t-1) + b(t)*(Y(t) - a*c*s(t-1));                                                              %由上一时刻状态的最优化估计s(t-1)得到当前时刻的最优化估计
    p(t) = p1(t) - c*b(t)*p1(t);                                                                                   %此时由一步估计得协方差得到此时刻最优化估计得协方差
end
% t = 1:N;
% figure(1);
% plot(t,dataTempMeasurement,'b',t,s,'r');
% legend('温度测量值','经过滤波后的温度估计值');
% xlabel('测量次序（/次）');
% ylabel('温度（/℃）');
% grid on;
% figure(2);
% plot(t,V);
% figure(3);
% plot(t,w);

