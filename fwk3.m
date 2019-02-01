clc;
clear all;
close all;
pause on;

% The result:
% https://media.giphy.com/media/NhrjAqODLq0tzjwHnN/giphy.gif


endLoc = [0;0;0;1]; % the 1 at the end allows us to do translation via matrix multiplication
endLoc2= [10;0;0;1]; % this is just for comparison later
% order: theta, alpha, r, d
P = zeros(0,4);
P = [P;0,90,6,0];
P = [P;0,90,6,0];
P = [P;0,0,6,0];

loopAmt = 359;
F = zeros(loopAmt, 4, size(P,1)+1);
for i=1:loopAmt
    P(1,1) = i;
    P(2,1) = i;
    P(3,1) = i;
    T = eye(4);
    for i1=1:size(P,1)
        theta = P(i1,1);
        alpha = P(i1,2);
        r = P(i1,3);
        d = P(i1,4);
        A = [
            cosd(theta),-sind(theta)*cosd(alpha),sind(theta)*sind(alpha),r*cosd(theta);
            sind(theta),cosd(theta)*cosd(alpha),-cosd(theta)*sind(alpha),r*sind(theta);
            0,sind(alpha),cosd(alpha),d;
            0,0,0,1
            ];
        T = T*A;
        F(i,:,i1+1) = transpose(T*endLoc);
    end
    disp("T*base");
    disp(T*endLoc);
    disp("T*base2");
    disp(T*endLoc2);
end
%pause(5);
i = 1;
showPath = true;
while i < loopAmt
    clf; % comment out to see range of motion
    hold on;
    for i1=size(F,3):-1:2
        plot3([F(i,1,i1), F(i,1,i1-1)],...
            [F(i,2,i1), F(i,2,i1-1)],...
            [F(i,3,i1), F(i,3,i1-1)]...
            )
    end
    xlim([-18,18]);
    ylim([-20,18]);
    zlim([-18,18]);
    %axis equal; % stops the oval shape but makes graph constantly scale
    view(3);
    pause(0.022); % 0.022 second delay
    i = i + 3;
end