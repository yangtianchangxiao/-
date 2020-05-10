%%单关节控制
%%本次控制为轨迹跟踪控制，包含恒定干扰
%%控制对象为实际的运动轨迹x(t),使其图像尽可能靠近目标轨迹xd(t)
%%xd(t)存在二阶导数
%%恒定干扰数值：t_dist=10
%% m=5 b=10 k=8
%%kv=2,kp=1
clear all
kv=2;
kp=1;
ki=0.0000001;
v=3;%%初始速度
m=5;%%初始化m b k
b=10;
k=8;
e=10*rand();%%实际x在输出时由于各种因素产生的随机干扰误差，本次没添加
err_sum=0;
err_dot=0;
err_last=0;
err=0;

t=0:0.01:10;
record=zeros(1,size(t,2)-1);%%记录实际x与目标轨迹xd的误差变化
x=zeros(1,size(t,2)-1);%%实际的x轨迹，初始化
t_dist=10;
num=2;
xd=2*t.*t+3*t;%%预设x的轨迹
f=m*4+b*(4*t+3)+k*xd+t_dist;%%模型法提供的力
for i=0.01:0.01:10
    err=xd(num)-x(num-1);
    err_dot=(err-err_last)/0.01;
    err_sum=err_sum+err;
    f_servo=4+kv*err_dot+kp*err+ki*err_sum;%%伺服法提供的力
    record(num-1)=err;
    x(num)=x(num-1)+0.5*f_servo*0.01^2+v*0.01%%print 实际轨迹x在各个时间点的值，初中物理v*t+0.5*a*t^2
    err_last=err;
    v=v+f_servo*0.01;
    num=num+1;
end
subplot(1,3,1)
plot(x,'r')
legend('实际运动图像')
subplot(1,3,2)
plot(xd,'g')
legend("理论运动图像")
subplot(1,3,3)
plot(record,'b')
legend("误差")

