%%���ؽڿ���
%%���ο���Ϊ�켣���ٿ��ƣ������㶨����
%%���ƶ���Ϊʵ�ʵ��˶��켣x(t),ʹ��ͼ�񾡿��ܿ���Ŀ��켣xd(t)
%%xd(t)���ڶ��׵���
%%�㶨������ֵ��t_dist=10
%% m=5 b=10 k=8
%%kv=2,kp=1
clear all
kv=2;
kp=1;
ki=0.0000001;
v=3;%%��ʼ�ٶ�
m=5;%%��ʼ��m b k
b=10;
k=8;
e=10*rand();%%ʵ��x�����ʱ���ڸ������ز������������������û���
err_sum=0;
err_dot=0;
err_last=0;
err=0;

t=0:0.01:10;
record=zeros(1,size(t,2)-1);%%��¼ʵ��x��Ŀ��켣xd�����仯
x=zeros(1,size(t,2)-1);%%ʵ�ʵ�x�켣����ʼ��
t_dist=10;
num=2;
xd=2*t.*t+3*t;%%Ԥ��x�Ĺ켣
f=m*4+b*(4*t+3)+k*xd+t_dist;%%ģ�ͷ��ṩ����
for i=0.01:0.01:10
    err=xd(num)-x(num-1);
    err_dot=(err-err_last)/0.01;
    err_sum=err_sum+err;
    f_servo=4+kv*err_dot+kp*err+ki*err_sum;%%�ŷ����ṩ����
    record(num-1)=err;
    x(num)=x(num-1)+0.5*f_servo*0.01^2+v*0.01%%print ʵ�ʹ켣x�ڸ���ʱ����ֵ����������v*t+0.5*a*t^2
    err_last=err;
    v=v+f_servo*0.01;
    num=num+1;
end
subplot(1,3,1)
plot(x,'r')
legend('ʵ���˶�ͼ��')
subplot(1,3,2)
plot(xd,'g')
legend("�����˶�ͼ��")
subplot(1,3,3)
plot(record,'b')
legend("���")

