 classdef Kinematics
    methods(Static)
        
        % forward kinematics (get the transformation T)
        function [T] = fwk(P)
                T = eye(4);
                for i1=size(P,1):-1:1
                    theta = P(i1,1);
                    alpha = P(i1,2);
                    r = P(i1,3);
                    d = P(i1,4);
                    % https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Denavit%E2%80%93Hartenberg_matrix
                    % called T on wikipedia
                    A = [
                        cosd(theta),-sind(theta)*cosd(alpha),sind(theta)*sind(alpha),r*cosd(theta);
                        sind(theta),cosd(theta)*cosd(alpha),-cosd(theta)*sind(alpha),r*sind(theta);
                        0,sind(alpha),cosd(alpha),d;
                        0,0,0,1
                        ];
                    % we need to transform link by link from end effector to the base
                    % So add it on
                    T = A*T;
                end
        end
        
        % inverse kinematics, only position
        % gradient descent-ish
        % (very bad way to do it, extremely easy to implement though)
        % it uses distance formula for the cost/error value
        function [newP] = invKPosition(P,target)
            z = [0;0;0;1];
            stepSize = 0.5;
            vars = P;
            loc0 = Kinematics.fwk(vars)*z;
            dist0 = Kinematics.distanceFormula(target,loc0);
            change = zeros(size(vars));
            for i=1:size(vars)
                newVars = vars;
                newVars(i) = newVars(i) + stepSize;
                loc1 = Kinematics.fwk(newVars)*z;
                change(i,1) = Kinematics.distanceFormula(target,loc1) - dist0;
            end
            newP = P - change.*stepSize;
        end
        
        % read invKPosition comments
        % modification: since distance just serves as cost/error, we can skip square root.
        function [newP] = invKPosition2(P,target)
            z = [0;0;0;1];
            stepSize = 0.5;
            vars = P;
            loc0 = Kinematics.fwk(vars)*z;
            % sq formula used because its an error, squaring isn't necessary
            dist0 = Kinematics.distanceSqFormula(target,loc0);
            change = zeros(size(vars));
            for i=1:size(vars)
                newVars = vars;
                newVars(i) = newVars(i) + stepSize;
                loc1 = Kinematics.fwk(newVars)*z;
                change(i,1) = Kinematics.distanceSqFormula(target,loc1) - dist0;
            end
            newP = P - change.*stepSize;
        end
        
    
        % unfinished
        % This gets the end P all at once
        % func is a handle to the inverse kinematics function that we want to use
        function [P] = invkPositionEndState(P,target, func)
            maxAllowedSteps = 1000;
            for i=1:maxAllowedSteps
                P = Kinematics.invKPosition2(P,target);
                if 0.1 > Kinematics.distanceFormula(Kinematics.fwk(P)*loc,target)
                    break
                end
            end
        end
    
        % attempt to make a more efficient cost function
        % doesn't really work, not worth it.
        function [cost] = cost1(v1,v2)
            cost  = sum(abs(v1-v2));
        end
        
        % distance formula
        % works for more than 3 dimensions if necessary
        function [distance] = distanceFormula(v1,v2)
            distance = sum((v1 - v2).^2).^(0.5);
        end
        
        % distance formula that doesn't square root at the end
        % used as a cost function
        function [distance] = distanceSqFormula(v1,v2)
            distance = sum((v1 - v2).^2);
        end
        
        % this might be broken
        % it is supposed to put a speed limit on the arm
        % When simulating, without this, the invk can move really fast.
        % which is bad for viewing the simulation and for training/ML if I ever do that.
        % When controlling the arm, without this, the servo moves super quick
        % and my popsicle robot arm breaks in half
        function P1 = simulateTravel(rps,tick,P1,P2)
            max = tick*360*rps;
            for i=1:size(P1,1)
                if abs(P2(i,1) - P1(i,1)) > max
                    if P2(i,1) > P1(i,1)
                        P1(i,1) = P1(i,1) + max;
                    else
                        P1(i,1) = P1(i,1) - max;
                    end
                else
                    P1(i,1) = P2(i,1);
                end
            end
        end
        
        % draw any arm in 3D using forward kinematics
        function draw(P)
            z = [0;0;0;1];
            hold on;
            F = zeros(4,size(P,1)+1);
            for i=1:size(P,1)
                F(:,i+1) = Kinematics.fwk(P(1:i,:))*z;
            end
            for i=2:size(F,2)
                plot3([F(1,i), F(1,i-1)],...
                    [F(2,i), F(2,i-1)],...
                    [F(3,i), F(3,i-1)]...
                    )
            end
            xlim([-18,18]); % these are just values i chose that fit my situation
            ylim([-18,18]);
            zlim([-18,18]);
            %axis equal; % stops the oval shape but makes graph constantly scale
            view(3);
        end
    end
 end
