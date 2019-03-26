close all;
clear all;
% Answer to Problem 1
Problem1;

% Answer to Problem 2(a) (b) (d)
Problem2;

% Answer to Problem 3 Note: need input to run Problem 3
jointAngle_ini = [3, 2];
jointVelocity_ini = [1, -1];
jointAcc_ini = [1,1];
Ini_states = [jointAngle_ini; jointVelocity_ini; jointAcc_ini];
Problem3(Ini_states);