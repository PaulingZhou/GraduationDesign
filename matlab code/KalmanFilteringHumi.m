function[s,N] = KalmanFilteringHumi(dataHumiMeasurement)
N = length(dataHumiMeasurement);                                                                                                           %��������Ϊ447��
% dataHumiMeasurement = importdata('dataOutputTemp.txt');                     %���������ݴ�'dataOutputTemp.txt'��ȡ��
% dataCount = 1:1:N;                                                                                      %�����꣬��ͼ��
% plot(dataCount,dataHumiMeasurement,'-');                                              %���δ���������ͼ��
w = 0.025*randn(1,N);                                                                                       %wΪ������������ͺ�ߵ�v�ڿ������������Ϊ��˹��������
w(1) = 0;                                                                                                           %����һ��1*N������������һ����Ϊ0��
dataHumiReal(1) = dataHumiMeasurement(1);                                           %ʵ���¶�ֵ������ֵΪ�����ĵ�һ��ֵ
a = 1;                                                                                                                 %aΪ״̬ת����������Ϊ�¶�����һʱ������һʱ��û�б仯����a=1
for k = 2:N                                                                                                         %ϵͳ״̬����,kʱ�̵�״̬����k-1ʱ��״̬����״̬ת���������
    dataHumiReal(k) = a*dataHumiReal(k-1) + w(k-1);                                  %����δ�ӿ��������ʴ˴����Կ�����U(k)
end
V = 0.35*randn(1,N);                                                                                         %��������
q1 = std(V);                                                                                                        %��������v�ı�׼��
Rvv = q1 ^ 2;                                                                                                      %��������v�ķ���
q2 = std(dataHumiReal);                                                                                  %�¶�ʵ��ֵ�ı�׼��
Rxx = q2 ^ 2;                                                                                                       %�¶�ʵ��ֵ�ķ���
q3 = std(w);                                                                                                         %��������w�ı�׼��
Rww = q3 ^ 2;                                                                                                      %��������w�ķ���
c = 1;                                                                                                                    %�����������ʵ��Ϊ�¶�ֵ��c = 1
Y = c*dataHumiMeasurement;                                                                          %���ⷽ��
p(1) = 0;                                                                                                               %��ʼ���Ż�����Э����                                                                                                        
s(1) =dataHumiMeasurement(1);                                                                      %s(1)��ʾΪ��ʼ���Ż����ƣ���ʼʱû�й��ƣ���ȡ����ֵ
for t = 2:N
    p1(t) = a.^2*p(t-1)+Rww;                                                                                  %p1Ϊһ�����Ƶ�Э�����ʽ��t-1ʱ�����Ż�����s��Э����õ�t-1ʱ�̵�tʱ��һ������Э����
    b(t) = c*p1(t)/(c.^2*p1(t)+Rvv);                                                                         %bΪ���������棬�������ʾΪ״̬����Э��������������Э����֮��
    s(t) = a*s(t-1) + b(t)*(Y(t) - a*c*s(t-1));                                                              %����һʱ��״̬�����Ż�����s(t-1)�õ���ǰʱ�̵����Ż�����
    p(t) = p1(t) - c*b(t)*p1(t);                                                                                   %��ʱ��һ�����Ƶ�Э����õ���ʱ�����Ż����Ƶ�Э����
end
% t = 1:N;
% figure(2);
% plot(t,dataHumiMeasurement,'b',t,s,'r');
% legend('�¶Ȳ���ֵ','�����˲�����¶ȹ���ֵ');
% xlabel('��������/�Σ�');
% ylabel('�¶ȣ�/�棩');
% grid on;
% figure(3);
% plot(t,V);
% figure(3);
% plot(t,w);

