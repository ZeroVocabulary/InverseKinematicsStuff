clc;
clear all;
close all;
pause on;

% generates an inverse kinematics dataset and saves it to files


loc = [0;0;0;1];
% order: theta, alpha, r, d
P = zeros(0,4);

% easyarm
P = [P;0,0,3,0];

% arm1
% P = [P;0,90,3,0];
% P = [P;0,90,3,0];
% P = [P;0,-90,3,0];
% P = [P;0,-90,3,0];
% P = [P;0,90,3,0];
% P = [P;0,90,3,0];
% P = [P;0,-90,3,0];
% P = [P;0,-90,3,0];
% P = [P;0,90,3,0];

% generate dataset
dataCount = 10000;
output = 360 * rand(size(P,1), dataCount); %randomly generate angles
input = zeros(4, dataCount);
for i=1:dataCount
    curr = P;
    curr(:,1) = output(:,i);
    input(:,i) = Kinematics.fwk(curr) * loc; % set input to end effector position
end

disp("done");
csvwrite('data/input.csv',transpose(input));
csvwrite('data/output.csv',transpose(output));

