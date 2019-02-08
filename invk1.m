clc;
clear all;
close all;
pause on;

pause(5);
loc = [0;0;0;1];
% order: theta, alpha, r, d
P = zeros(0,4);

% arm1
P = [P;0,90,3,0];
P = [P;0,90,3,0];
P = [P;0,-90,3,0];
P = [P;0,-90,3,0];
P = [P;0,90,3,0];
P = [P;0,90,3,0];
P = [P;0,-90,3,0];
P = [P;0,-90,3,0];
P = [P;0,90,3,0];

% arm2 (randomly generated)
% disp("Arm randomly generated, invk might not be possible depending on random generation")
% P = [P;0,90*rand,5*rand(),0];
% P = [P;0,90*rand(),5*rand(),0];
% P = [P;0,-90*rand(),5*rand(),0];
% P = [P;0,-90*rand(),5*rand(),0];
% P = [P;0,90*rand(),5*rand(),0];
% P = [P;0,90*rand(),5*rand(),0];
% P = [P;0,-90*rand(),5*rand(),0];
% P = [P;0,-90*rand(),5*rand(),0];
% P = [P;0,90*rand(),5*rand(),0];

target = [-8;3;12;1];
%target = [0.1;0.1;0.1;1];


i = 2;
steps = 1000;
stepsDone = steps;
errorList = zeros(steps,1);
errorList(1) = 5;
F = zeros(size(P,1),4,steps);
F(:,:,1) = P;
scriptStartTime = clock;
tick = 0.1;
P(1,3) = 5;
while i < steps && errorList(i - 1) > 0.1
    P = Kinematics.simulateTravel(0.05, tick, P, Kinematics.invKPosition2(P,target));
    % inverse kinmatics goes too fast on its own to see sometimes so simulateTravel is used instead
    %P = Kinematics.invKPosition2(P,target);
    F(:,:,i) = P;
    errorList(i) = Kinematics.distanceFormula(Kinematics.fwk(P)*loc,target);
    
    i = i + 1;
end
stepsDone = i;
disp("Runtime:" + etime(clock, scriptStartTime) + "    steps: " + stepsDone);


for i=1:2:stepsDone-1
    clf;
    plot3([0, target(1)],... % This plots a line to the target point
    [0, target(2)],...
    [0, target(3)]...
    )
    Kinematics.draw(F(:,:,round(i)));
    xlim([-25,25]);
    ylim([-25,25]);
    zlim([-25,25]);
    %axis equal; % stops the oval shape but makes graph constantly scale
    view(3);
    pause(tick);
end


% Plot the error over time
figure();
stem(errorList);

% Plot the error for the last 20% of steps
figure();
stem(errorList(round(stepsDone*0.8):stepsDone));
