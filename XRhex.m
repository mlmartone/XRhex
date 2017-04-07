%%% XRhex Robot Class %%%
%Defines an XRhex robot as a hexapod robot of X-Actuators
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
        %Constructor method for creating a new object
        function robot = XRhex(group1,group2)
            robot.group1 = group1;
            robot.group2 = group2;
            robot.fbk1 = robot.group1.getNextFeedback();
            robot.fbk2 = robot.group2.getNextFeedback();
        end
        
        %Verifies position error is within acceptable tolerance
        function out = checkPosError(robot,fbk)
            error = abs(fbk.position-fbk.positionCmd);
            out = max(error) < robot.maxErr;
        end
        
        %Moves the legs of the XRhex to a given position based on leg
        %group, with no constraints on time or torque, but verifying
        %position error before returning
        function moveLegsToPosGroup(robot,pos1,pos2)
            robot.moveLegsToPos([pos1 pos2 pos1 pos2 pos1 pos2]);
        end
        
        %Moves the legs of the XRhex to a given set of positions
        %individually, with no constraints on time or torque, but verifying
        %position error before returning
        function moveLegsToPos(robot,pos)
            robot.cmd1 = CommandStruct();
            robot.cmd2 = CommandStruct();
            robot.fbk1 = robot.group1.getNextFeedback();
            robot.fbk2 = robot.group2.getNextFeedback();
            curPos = [robot.fbk1.position(1) robot.fbk2.position(1) ...
                robot.fbk1.position(2) -robot.fbk2.position(2) ...
                -robot.fbk1.position(3) -robot.fbk2.position(3)];
            pos = mod(pos,2*pi)+floor(curPos/(2*pi))*2*pi;
            pos = pos + (curPos-robot.maxErr > pos)*2*pi;
            start = tic;
            while ~robot.checkPosError(robot.fbk1) || ...
                    ~robot.checkPosError(robot.fbk2)
                robot.cmd1.position = [pos(1) pos(3) -pos(5)];
                robot.group1.set(robot.cmd1);
                pause(.05);
                robot.cmd2.position = [pos(2) -pos(4) -pos(6)];
                robot.group2.set(robot.cmd2);
                pause(.05);
                robot.fbk1 = robot.group1.getNextFeedback();
                robot.fbk2 = robot.group2.getNextFeedback();
                if(toc(start) > 5)
                    disp('Stale Module Error');
                    break
                end
            end
        end
        
        %Moves the legs of the robot through given trajectory points as leg
        %groups
        function followLegTrajGroup(robot,trajPoints,startPt,endPt)
            robot.followLegTraj([trajPoints(1,:); trajPoints(2,:);...
                trajPoints(1,:); trajPoints(2,:); trajPoints(1,:);...
                trajPoints(2,:)],startPt,endPt);
        end
        
        %Moves the legs of the robot through given trajectory points
        %individially
        function followLegTraj(robot,trajPoints,startPt,endPt)
            robot.fbk1 = robot.group1.getNextFeedback();
            robot.fbk2 = robot.group2.getNextFeedback();
            curPos = [robot.fbk1.position(1) robot.fbk2.position(1) ...
                robot.fbk1.position(2) -robot.fbk2.position(2) ...
                -robot.fbk1.position(3) -robot.fbk2.position(3)]';
            trajPoints = trajPoints - ...
                repmat(floor(trajPoints(:,1)/(2*pi))*2*pi,1,size(trajPoints,2));
            trajPoints = trajPoints+...
                repmat(floor(curPos/(2*pi))*2*pi,1,size(trajPoints,2));
            trajPoints = trajPoints + ...
                repmat((curPos-robot.maxErr > trajPoints(:,1))*2*pi,...
                1,size(trajPoints,2));
            for i = startPt:1:endPt-1
                pos = trajPoints(:,i);
                robot.cmd1.position = [pos(1) pos(3) -pos(5)];
                robot.group1.set(robot.cmd1);
                pause(robot.pauseTime);
                robot.cmd2.position = [pos(2) -pos(4) -pos(6)];
                robot.group2.set(robot.cmd2);
                pause(robot.pauseTime);
            end
            robot.fbk1 = robot.group1.getNextFeedback();
            robot.fbk2 = robot.group2.getNextFeedback();
        end
        
        %Generates a smooth trajectory of positions for the legs to pass
        %through given waypoints and desired timesteps
        function traj = generateLegTrajGroup(robot,waypoints,times)
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
        
        %Generates a smooth trajectory of positions for the legs to pass
        %through given waypoints and desired timesteps
        function traj = generateLegTraj(robot,waypoints,times,speeds)
            times = times-times(1);
            numWaypoints = size(waypoints,2);
            trajStart = times.*robot.freq+1;
            traj = zeros(6,trajStart(numWaypoints));
            if(nargin == 3)
                speeds = zeros(6,numWaypoints);
            end
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
        
        %Moves the robot from a leg up starting position to a standing
        %position
        function standUp(robot)
            standUpTraj = robot.generateLegTrajGroup([pi,2*pi;pi,2*pi],[0,1]);
            robot.followLegTrajGroup(standUpTraj,1,size(standUpTraj,2));
        end
        
        %Moves the robot forward by one step using the tripod gait
        function takeStep(robot,stepSize,stepTime,gaitName)
            robot.fbk1 = robot.group1.getNextFeedback();
            robot.fbk2 = robot.group2.getNextFeedback();
            curPos = [robot.fbk1.position(1);robot.fbk2.position(1);...
                robot.fbk1.position(2); -robot.fbk2.position(2);...
                -robot.fbk1.position(3); -robot.fbk2.position(3)];
            pos1 = 2*pi*ceil(curPos/(2*pi)) + ...
                stepSize*[1; -1; 1; -1; 1; -1];
            pos2 = pos1 + [2*pi-2*stepSize; 2*stepSize; 2*pi-2*stepSize;...
                2*stepSize; 2*pi-2*stepSize; 2*stepSize];
            stepPoints = [curPos,pos1,pos2];
            stepTimes = linspace(0,stepTime,size(stepPoints,2));
            speeds = zeros(6,3);
            if(strcmp(gaitName,'run'))
                dTh = [stepPoints(:,2)-stepPoints(:,1)...
                    stepPoints(:,3)-stepPoints(:,2)];
                dt = [stepTimes(2)-stepTimes(1) stepTimes(2)-stepTimes(1)];
                speeds = dTh./repmat(dt,6,1);
                speeds = [zeros(6,1) speeds(:,1) speeds(:,2)];
            end
            walkTraj = robot.generateLegTraj(stepPoints,stepTimes,speeds);
            robot.followLegTraj(walkTraj,1,size(walkTraj,2));
        end
        
        %Moves the robot forward by one step using the tripod gait
        function takeStepWave(robot,stepSize,stepTime,gaitName)
            robot.fbk1 = robot.group1.getNextFeedback();
            robot.fbk2 = robot.group2.getNextFeedback();
            curPos = [robot.fbk1.position(1);robot.fbk2.position(1);...
                robot.fbk1.position(2); -robot.fbk2.position(2);...
                -robot.fbk1.position(3); -robot.fbk2.position(3)];
            pos1 = 2*pi*ceil(curPos/(2*pi)) + ...
                stepSize*[-1; 0; 1; 1; 0; -1];
            pos2 = pos1 + [stepSize; stepSize; 2*pi-2*stepSize;...
                2*pi-2*stepSize; stepSize; stepSize];
            pos3 = pos2 + [stepSize; 2*pi-2*stepSize; stepSize;...
                stepSize; 2*pi-2*stepSize; stepSize];
            stepPoints = [curPos pos1 pos2 pos3];
            stepTimes = linspace(0,stepTime,size(stepPoints,2));
            speeds = zeros(6,4);
            if(strcmp(gaitName,'run'))
                dTh = [stepPoints(:,2)-stepPoints(:,1)...
                    stepPoints(:,3)-stepPoints(:,2)];
                dt = [stepTimes(2)-stepTimes(1) stepTimes(2)-stepTimes(1)];
                speeds = dTh./repmat(dt,6,1);
                speeds = [zeros(6,1) speeds(:,1) speeds(:,2)];
            end
            walkTraj = robot.generateLegTraj(stepPoints,stepTimes,speeds);
            robot.followLegTraj(walkTraj,1,size(walkTraj,2));
        end
    end
end