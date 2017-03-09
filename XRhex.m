classdef XRhex
    properties
        maxErr = .02; %rad
        freq = 100; %Hz
        pauseTime = 1/(2*100); %s
        group1;
        group2;
        cmd1 = CommandStruct();
        cmd2 = CommandStruct();
        fbk1;
        fbk2;
    end
    
    methods
        function robot = XRhex(group1,group2)
            robot.group1 = group1;
            robot.group2 = group2;
            robot.fbk1 = robot.group1.getNextFeedback();
            robot.fbk2 = robot.group2.getNextFeedback();
        end
        
        function out = checkPosError(robot,fbk)
            error = abs(fbk.position-fbk.positionCmd);
            out = max(error) < robot.maxErr;
        end
        
        function moveLegsToPos(robot,pos1,pos2)
            robot.cmd1 = CommandStruct();
            robot.cmd2 = CommandStruct();
            robot.fbk1 = robot.group1.getNextFeedback();
            robot.fbk2 = robot.group2.getNextFeedback();
            while ~robot.checkPosError(robot.fbk1) || ...
                    ~robot.checkPosError(robot.fbk2)
                robot.cmd1.position = [pos1 pos1 -pos1];
                robot.group1.set(robot.cmd1);
                pause(.05);
                robot.cmd2.position = [pos2 -pos2 -pos2];
                robot.group2.set(robot.cmd2);
                pause(.05);
                robot.fbk1 = robot.group1.getNextFeedback();
                robot.fbk2 = robot.group2.getNextFeedback();
            end
        end
        
        function followLegTraj(robot,trajPoints,startPt,endPt)
            for i = startPt:1:endPt-1
                pos = trajPoints(:,i);
                robot.cmd1.position = [pos(1) pos(1) -pos(1)];
                robot.group1.set(robot.cmd1);
                pause(robot.pauseTime);
                robot.cmd2.position = [pos(2) -pos(2) -pos(2)];
                robot.group2.set(robot.cmd2);
                pause(robot.pauseTime);
            end
            robot.fbk1 = robot.group1.getNextFeedback();
            robot.fbk2 = robot.group2.getNextFeedback();
        end
        
        function traj = generateLegTraj(robot,waypoints,times)
            times = times-times(1);
            numWaypoints = size(waypoints,2);
            trajStart = times.*robot.freq+1;
            traj = zeros(2,trajStart(numWaypoints));
            for j = 1:1:numWaypoints-1
                for i = 1:1:2
                    [a5,a4,a3,a2,a1,a0] = robot.createTraj5(...
                        waypoints(i,j),waypoints(i,j+1),0,0,0,0,...
                        times(j),times(j+1));
                    p = [a5,a4,a3,a2,a1,a0];
                    t = 0:1/robot.freq:times(j+1)-times(j);
                    traj(i,trajStart(j):trajStart(j+1)) = polyval(p,t);
                end
            end
        end
        
        function [a5,a4,a3,a2,a1,a0] = createTraj5(~,theta0,thetaf,...
                thetad0,thetadf,thetadd0,thetaddf,tstart,tfinal)
            %createTraj5 function adapted from code by Reza Ahmadzadeh
            %https://www.mathworks.com/matlabcentral/fileexchange/...
            %40278-trajectory-generation--3rd---5th-orders-
            T = tfinal - tstart;
            a0 = theta0;
            a1 = thetad0;
            a2 = 0.5 * thetadd0;
            a3 =(1/(2*T^3)) * (20 * (thetaf-theta0) - ...
                (8*thetadf+12*thetad0)*T - (3*thetaddf-thetadd0)*T^2);
            a4 =(1/(2*T^4)) * (30 * (theta0-thetaf) + ...
                (14*thetadf+16*thetad0)*T + (3*thetaddf-2*thetadd0)*T^2);
            a5 =(1/(2*T^5)) * (12 * (thetaf-theta0) - ...
                6*(thetadf+thetad0)*T - (thetaddf-thetadd0)*T^2);
        end
        
        function holdPos(robot,time)
            startTime = tic;
            robot.fbk1 = robot.group1.getNextFeedback();
            robot.fbk2 = robot.group2.getNextFeedback();
            robot.cmd1.position = robot.fbk1.position;
            robot.cmd2.position = robot.fbk2.position;
            while toc(startTime) < time
                robot.group1.set(robot.cmd1);
                pause(robot.pauseTime);
                robot.group2.set(robot.cmd2);
                pause(robot.pauseTime);
            end
        end
    end
end