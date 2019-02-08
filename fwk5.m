clc;
clear all;
close all;
pause on;


loc = [0;0;0;1];
% order: theta, alpha, r, d
P = zeros(0,4);

% arm1
P = [P;0,90,3,0];
P = [P;0,90,3,0];
P = [P;0,90,3,0];
P = [P;0,90,3,0];
P = [P;0,90,3,0];
P = [P;0,90,3,0];

% armReal
% P = [P;90,90,1,35*3];
% P = [P;0,0,115,29];
% P = [P;0,180,107,0];

loopAmt = 360;
F = zeros(loopAmt, 4, size(P,1)+1);
for i=1:3:loopAmt % step size of 3 just to run faster.
    % set each angle value to i
    % (their angles go from 0 to 360 degrees).
    for i1=1:size(P,1)
        P(i1,1) = i;
    end
    clf;
    % fwk done in draw function.
    Kinematics.draw(P);
    pause(0.05);
end