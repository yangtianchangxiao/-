clear all;
clc;
L1=Link([0,0.4605,0,0,0],'modified');
L2=Link([0,0.0855,0.350,0,0],'modified');
L3=Link([0,0,0.250,0,1],'modified');
robot=SerialLink([L1 L2 L3],'name','assignment1');
robot.plot([0 0 0],'workspace',[-1 1 -1 1 -1 1])
robot.display()
