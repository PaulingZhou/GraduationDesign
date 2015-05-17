clear
N = 447;                                                                                                           %��������Ϊ447��
dataTempMeasurement = importdata('dataOutputTemp.txt');                     %���������ݴ�'dataOutputTemp.txt'��ȡ��
% dataCount = 1:1:N;                                                                                          %�����꣬��ͼ��
% plot(dataCount,dataTempMeasurement,'-');                                                  %���δ����������ͼ��
w(1) = 0;                                                                                                           %����һ��1*N������������һ����Ϊ0��
w = randn(1,N);                                                                                                %wΪ������������ͺ�ߵ�v�ڿ������������Ϊ��˹��������
dataTempReal(1) = 25;                                                                                   %ʵ���¶�ֵ������ֵΪ25��
a = 1;                                                                                                                 %aΪ״̬ת����������Ϊ�¶�����һʱ������һʱ��û�б仯����a=1
for k = 2:N                                                                                                         %ϵͳ״̬����,kʱ�̵�״̬����k-1ʱ��״̬����״̬ת���������
    dataTempReal(k) = a*dataTempReal(k-1) + w(k-1);                                  %����δ�ӿ��������ʴ˴�����
end
V = randn(1,N);                                                                                                  %��������
q1 = std(V);                                                                                                        %��������v�ı�׼��
Rvv = q1 ^ 2;                                                                                                      %��������v�ķ���
q2 = std(dataTempReal);                                                                                  %�¶�ʵ��ֵ�ı�׼��
Rxx = q2 ^ 2;                                                                                                       %�¶�ʵ��ֵ�ķ���
q3 = std(w);                                                                                                         %��������w�ı�׼��
Rww = q3 ^ 2;                                                                                                      %��������w�ķ���
c = 1;                                                                                                                    %�����������ʵ��Ϊ�¶�ֵ��c = 1
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
