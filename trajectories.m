clear all
global l1 l2 d1 d2
l1=0.35;
l2=0.25;
d1=0.4605;
d2=0.0855;




coordinate1=input('输入初始坐标');
x1=coordinate1(1);
y1=coordinate1(2);
z1=coordinate1(3);
if (sqrt(x1^2+y1^2)>(l1+l2)||sqrt(x1^2+y1^2)<(l1-l2))
    fprintf("坐标超范围")
    return 
end
coordinate2=input('输入变化坐标');
x2=coordinate2(1);
y2=coordinate2(2);
z2=coordinate2(3);
if (sqrt(x2^2+y2^2)>l1+l2||sqrt(x2^2+y2^2)<l1-l2)
    fprintf("坐标超范围")
    return 
end
time=input('输入运动时间');
[theta1_1,theta2_1,z1]=converse(x1,y1,z1);
%%随便假定一种初始状态，本例假设theta2为正
%%求变化后的角度，设两个机械臂转动角度的损耗权值均为1.即转动同等重要，我们在规划时考虑使两个关节的转动角度之和最小
[theta1_2,theta2_2,z2]=converse(x2,y2,z2);

array1=theta1_2-theta1_1(1);
array2=theta2_2-theta2_1(1);
[num1,index1]=min(abs(array1));
[num2,index2]=min(abs(array2));


[s1,v1,a1]=lspb(theta1_1(1),theta1_2(index1),linspace(0,time,100));

[s2,v2,a2]=lspb(theta2_1(1),theta2_2(index2),linspace(0,time,100));
t=linspace(0,time,100);

%%动画部分
L1=Link([0,0.4605,0,0,0],'modified');
L2=Link([0,0.0855,0.350,0,0],'modified');
L3=Link([0,0,0.250,0,1],'modified');
robot=SerialLink([L1 L2 L3],'name','assignment1');
init_ang = [theta1_1(1),theta2_1(1) z1];
targ_ang = [theta1_2(index1) theta2_2(index2) z2];
[q,qd,qdd]=jtraj(init_ang,targ_ang,100);
robot.plot(q,'workspace',[-1 1 -1 1 -1 1])

%%数据打印
figure(1)
plot(t,s1,'R');
hold on
plot(t,v1,'G')
hold on
plot(t,a1,'B')
legend('theta1','theta1-dot','theta1-double-dot')
figure(2)
plot(t,s2,'R');
hold on
plot(t,v2,'G')
hold on
plot(t,a2,'B')
legend('theta2','theta2-dot','theta2-double-dot')
figure(3)
plot(t,linspace(z1,z2,100))
legend("最后的移动关节运动情况")


%%反解关节角

function [theta1,theta2,d3]=converse(x,y,z)
l1=0.35;
l2=0.25;
d1=0.4605;
d2=0.0855;
theta2=[acos((x^2+y^2-l1^2-l2^2)/2/l1/l2) -acos((x^2+y^2-l1^2-l2^2)/2/l1/l2)];
beta=atan2(y,x);
fai=acos((l1^2+x^2+y^2-l2^2)/(2*l1*sqrt(x^2+y^2)));
theta1=[beta-fai beta+fai];
d3=z-d1-d2;

end