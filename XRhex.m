%% XRhex Robot Class
%Defines an XRhex robot as a hexapod robot of X-Actuators
%Updated 4/11/17
classdef XRhex
    properties
        maxErr = .05; %rad
        freq = 100; %Hz
        pauseTime = 1/(2*100); %s
        group;
        cmd = CommandStruct();
        fbk;
    end
    
    methods
        %Constructor method for creating a new robot object
        function robot = XRhex(group)
            robot.group = group;
            robot.fbk = robot.group.getNextFeedback();
        end
        
        %Verifies position error is within acceptable tolerance
        function out = checkPosError(robot,fbk)
            error = abs(fbk.position-fbk.positionCmd);
            out = max(error) < robot.maxErr;
        end
        
        %Moves the legs to a given set of positions with no constraints on
        %time or torque, but verifying position error before returning
        function moveLegsToPos(robot,pos)
            %Adjust the given position to be within +1 rotation of the
            %current position
            robot.fbk = robot.group.getNextFeedback();
            curPos = robot.fbk.position.*[1 1 1 -1 -1 -1];
            pos = mod(pos,2*pi)+floor(curPos/(2*pi))*2*pi;
            pos = pos + (curPos-robot.maxErr > pos)*2*pi;
            posDiff = pos-curPos;
            start = tic;
            ramp = 0;
            %Command the position until the error is satisfied
            while ~robot.checkPosError(robot.fbk)
                ramp = ramp + .02;
                if ramp >= 1; ramp = 1; end
                robot.cmd.position = (curPos + posDiff*ramp).*...
                    [1 1 1 -1 -1 -1];
                robot.group.set(robot.cmd);
                pause(robot.pauseTime);
                robot.fbk = robot.group.getNextFeedback();
                %Break out of infinite loops caused by unresponsive modules
                if(toc(start) > 3); 
                    break
                    error('Unresponsive Module Error');
                end
            end
        end
                
        %Moves the legs of the robot through given trajectory points
        function followLegTraj(robot,trajPoints,startPt,endPt)
            %Adjust the given trajectory to be within +1 rotation of the
            %current position
            robot.fbk = robot.group.getNextFeedback();
            curPos = robot.fbk.position'.*[1 1 1 -1 -1 -1]';
            trajPoints = trajPoints - repmat(...
                floor(trajPoints(:,1)/(2*pi))*2*pi,1,size(trajPoints,2));
            trajPoints = trajPoints+...
                repmat(floor(curPos/(2*pi))*2*pi,1,size(trajPoints,2));
            trajPoints = trajPoints + ...
                repmat((curPos-robot.maxErr > trajPoints(:,1))*2*pi,...
                1,size(trajPoints,2));
            %Iterate through commnanding the trajectory points
            for i = startPt:1:endPt-1
                pos = trajPoints(:,i);
                robot.cmd.position = pos'.*[1 1 1 -1 -1 -1];
                robot.group.set(robot.cmd);
                pause(robot.pauseTime);
            end
        end
        
        %Generates a smooth trajectory of positions for the legs to pass
        %through given waypoints, at desired times, with given speeds
        function traj = generateLegTraj(robot,waypoints,times,speeds)
            times = times-times(1);
            numWaypoints = size(waypoints,2);
            trajStart = times.*robot.freq+1;
            traj = zeros(6,trajStart(numWaypoints));
            %Check if speeds were given, if not set them to 0
            if(nargin == 3)
                speeds = zeros(6,numWaypoints);
            end
            %Iterate through the waypoints generating the trajectory
            for j = 1:1:numWaypoints-1
                for i = 1:1:6
                    [a5,a4,a3,a2,a1,a0] = robot.createTraj5(...
                        waypoints(i,j),waypoints(i,j+1),speeds(i,j),...
                        speeds(i,j+1),0,0,times(j),times(j+1));
                    p = [a5,a4,a3,a2,a1,a0];
                    t = 0:1/robot.freq:times(j+1)-times(j);
                    traj(i,trajStart(j):trajStart(j+1)) = polyval(p,t);
                end
            end
        end
        
        %Creates a minimum jerk trajectory given position, velocity, and
        %acceleration at different timesteps
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
        
        %Pauses the XRhex's motion in place, ensuring commands persist for
        %alloted time
        function holdPos(robot,time)
            startTime = tic;
            robot.fbk = robot.group.getNextFeedback();
            robot.cmd.position = robot.fbk.position;
            %Command the current position for the given time
            while toc(startTime) < time
                robot.group.set(robot.cmd);
                pause(robot.pauseTime);
            end
        end
        
        %Moves the robot from a leg up starting position to a standing
        %position
        function standUp(robot)
            %Find the current position and the next posible up position
            robot.fbk = robot.group.getNextFeedback();
            curPos = robot.fbk.position'.*[1 1 1 -1 -1 -1]';
            goalPos = 2*pi*ceil(curPos/(2*pi));
            %Generate and execute a smooth trajectory
            standUpTraj = robot.generateLegTraj([curPos,goalPos],[0,1]);
            robot.followLegTraj(standUpTraj,1,size(standUpTraj,2));
        end
        
        %Moves the robot forward by one step using the selected gait
        function takeStep(robot,stepSize,stepTime,gaitName)
            switch gaitName
                case 'tripod'
                    robot.takeStepTripod(stepSize,stepTime);
                case 'wave'
                    robot.takeStepWave(stepSize,stepTime);
            end
        end
        
        %Moves the robot backwards one step using the tripod gait
        function takeStepBackwards(robot,stepSize,stepTime)
            %Generate the waypoints with timesteps for one step
            robot.fbk = robot.group.getNextFeedback();
            curPos = robot.fbk.position'.*[1 1 1 -1 -1 -1]';
            pos1 = 2*pi*round(curPos/(2*pi)) - ...
                [stepSize(1); 2*pi-stepSize(1); stepSize(1);...
                2*pi-stepSize(2); stepSize(2); 2*pi-stepSize(2)];
            pos2 = pos1 - [2*pi-2*stepSize(1); 2*stepSize(1); ...
                2*pi-2*stepSize(1); 2*stepSize(2); 2*pi-2*stepSize(2);...
                2*stepSize(2)];
            stepPoints = [curPos,pos1,pos2];
            stepTimes = linspace(0,stepTime,size(stepPoints,2));
            speeds = zeros(6,3);
            %Generate and execute the trajectory
            walkTraj = robot.generateLegTraj(stepPoints,stepTimes,speeds);
            robot.followLegTraj(walkTraj,1,size(walkTraj,2));
        end
        
        %Moves the robot forward by one step using the tripod gait
        function takeStepTripod(robot,stepSize,stepTime)
            %Generate the waypoints with timesteps for one step
            robot.fbk = robot.group.getNextFeedback();
            curPos = robot.fbk.position'.*[1 1 1 -1 -1 -1]';
            pos1 = 2*pi*round(curPos/(2*pi)) + ...
                [stepSize(1); 2*pi-stepSize(1); stepSize(1);...
                2*pi-stepSize(2); stepSize(2); 2*pi-stepSize(2)];
            pos2 = pos1 + [2*pi-2*stepSize(1); 2*stepSize(1); ...
                2*pi-2*stepSize(1); 2*stepSize(2); 2*pi-2*stepSize(2);...
                2*stepSize(2)];
            stepPoints = [curPos,pos1,pos2];
            stepTimes = linspace(0,stepTime,size(stepPoints,2));
            speeds = zeros(6,3);
            %Generate and execute the trajectory
            walkTraj = robot.generateLegTraj(stepPoints,stepTimes,speeds);
            robot.followLegTraj(walkTraj,1,size(walkTraj,2));
        end
        
        %Moves the robot forward by one step using the tripod gait
        function takeStepWave(robot,stepSize,stepTime)
            %Generate the waypoints with timesteps for one step
            robot.fbk = robot.group.getNextFeedback();
            curPos = robot.fbk.position'.*[1 1 1 -1 -1 -1]';
            pos1 = 2*pi*ceil(curPos/(2*pi)) + ...
                [-stepSize(1); 0; stepSize(1); stepSize(2); 0;...
                -stepSize(2)];
            pos2 = pos1 + [stepSize(1); stepSize(1); 2*pi-2*stepSize(1);...
                2*pi-2*stepSize(2); stepSize(2); stepSize(2)];
            pos3 = pos2 + [stepSize(1); 2*pi-2*stepSize(1); stepSize(1);...
                stepSize(2); 2*pi-2*stepSize(2); stepSize(2)];
            stepPoints = [curPos pos1 pos2 pos3];
            stepTimes = linspace(0,stepTime,size(stepPoints,2));
            speeds = zeros(6,4);
            %Generate and execute the trajectory
            walkTraj = robot.generateLegTraj(stepPoints,stepTimes,speeds);
            robot.followLegTraj(walkTraj,1,size(walkTraj,2));
        end
        
        %Moves the robot forward by one step using the tripod gait
        function takeStepTripodRun(robot,stepSize,stepTime)
            %Generate the waypoints with timesteps for one step
            robot.fbk = robot.group.getNextFeedback();
            curPos = robot.fbk.position'.*[1 1 1 -1 -1 -1]';
            pos1 = 2*pi*round(curPos/(2*pi)) + ...
                [stepSize(1); 2*pi-stepSize(1); stepSize(1);...
                2*pi-stepSize(2); stepSize(2); 2*pi-stepSize(2)];
            pos2 = pos1 + [2*pi-2*stepSize(1); 2*stepSize(1); ...
                2*pi-2*stepSize(1); 2*stepSize(2); 2*pi-2*stepSize(2);...
                2*stepSize(2)];
            stepPoints = [curPos,pos1,pos2];
            stepTimes = linspace(0,stepTime,size(stepPoints,2));
            %dTh = [stepPoints(:,2)-stepPoints(:,1)...
            %stepPoints(:,3)-stepPoints(:,2)];
            %dt = [stepTimes(2)-stepTimes(1) stepTimes(2)-stepTimes(1)];
            speeds = ones(6,3)*1;%[zeros(6,1) dTh./repmat(dt,6,1)];
            %Generate and execute the trajectory
            walkTraj = robot.generateLegTraj(stepPoints,stepTimes,speeds);
            robot.followLegTraj(walkTraj,1,size(walkTraj,2));
        end
    end
end