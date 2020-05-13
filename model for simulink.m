clear all;
clc;
l1 = 0.35;
d2= 0.4605;
l2 = 0.25;
d_offset = 0.0855;
L(1) = Link([0,d2,0,0,0],'modified');
L(2) = Link([0,d_offset,l1,0,0],'modified');
L(3) = Link([0,0,l2,0,0],'modified');
L(4) = Link([0,0,0,0,1],'modified');
L(3).qlim = [0,0.6];
L(1).r = [0.35, 0, 0.4605];
L(2).r = [0.6, 0, 0.546];
L(3).r = [0.6, 0, 0.546];
L(4).r = [0.6, 0, 0.5];
% Mass
L(1).m = 10;
L(2).m = 30;
L(3).m = 2;
L(4).m = 0.5;
% Inertia
L(1).I = [0.02, 0, 0; 0, 0.21, 0; 0, 0, 0.23];
L(2).I = [0.27, 0, 0; 0, 0.61, 0; 0, 0, 0.43];
L(3).I = [0.06, 0, 0; 0, 0.06, 0; 0, 0, 0.00];
L(4).I = [0.0005, 0, 0; 0, 0.0005, 0; 0, 0, 0.0002]; % suppose r = 0.03, h = 0.1
% Motor inertia
L(1).Jm = 0;
L(2).Jm = 0;
L(3).Jm = 0;
L(4).Jm = 0;
% Motor friction
L(1).B = 0.0001;
L(2).B = 0.0001;
L(3).B = 0.0001;
L(4).B = 0.0001;
% Gear ratio
L(1).G = 9.8
L(2).G = 9.8;
L(3).G = 9.8;
L(4).G = 9.8;
robot_model=SerialLink([L(1) L(2) L(3) L(4)],'name','assignment1');
robot_model.plot([0 0 0 0],'workspace',[-1 1 -1 1 -1 1])
robot_model.display()
