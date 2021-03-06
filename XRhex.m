%% XRhex Robot Class
%Defines an XRhex robot as a hexapod robot of X-Actuators
%Updated 10/29/17
classdef XRhex
    properties
        maxErr = .1; %rad
        %ADD SOMETHING HERE TO PREVENT SPINOUTS (INSTEAD OF MAXERR)
        freq = 100; %Hz
        pauseTime = 1/(2*100); %s
        group;
        cmd = CommandStruct();
        fbk;
        directionFlip = [-1 -1 -1 1 1 1];
    end
    
    methods
        %Constructor method for creating a new robot object
        function robot = XRhex(group)
            robot.group = group;
            robot.fbk = robot.group.getNextFeedback();
        end
        
        %Verifies position error is within acceptable tolerance of
        %commanded position
        function out = checkPosError(robot,fbk)
            error = abs(fbk.position-fbk.positionCmd);
            out = max(error) < robot.maxErr;
        end
        
        %Verifies position error is within acceptable tolerance of
        %given position vector
        function out = checkPosAgainstGiven(robot,fbk,given)
            error = abs(fbk.position-given);
            out = max(error) < robot.maxErr;
        end
        
        %Moves the legs to a given set of positions with no constraints on
        %time or torque, but verifying position error before returning
        function moveLegsToPos(robot,pos)
            %Adjust the given position to be within +1 rotation of the
            %current position
            robot.fbk = robot.group.getNextFeedback();
            curPos = robot.fbk.position.*robot.directionFlip;
            pos = mod(pos,2*pi)+floor(curPos/(2*pi))*2*pi;
            pos = pos + (curPos-robot.maxErr > pos)*2*pi;
            posDiff = pos-curPos;
            start = tic;
            ramp = 0;
            %Command the position until the error is satisfied
            while ~robot.checkPosAgainstGiven(robot.fbk,pos)
                ramp = ramp + .02;
                if ramp >= 1; ramp = 1; end
                robot.cmd.position = (curPos + posDiff*ramp).*...
                    robot.directionFlip;
                robot.group.set(robot.cmd);
                pause(robot.pauseTime);
                robot.fbk = robot.group.getNextFeedback();
                %Break out of infinite loops caused by unresponsive modules
                if(toc(start) > 3); 
                    break;
                    %error('Unresponsive Module Error');
                end
            end
        end
        
        %Moves the legs to a given set of positions with no constraints on
        %time or torque, but verifying position error before returning
        function biDirMoveLegsToPos(robot,pos)
            %Adjust the given position to be within +/-1 rotation of the
            %current position
            robot.fbk = robot.group.getNextFeedback();
            curPos = robot.fbk.position.*robot.directionFlip;
            pos = mod(pos,2*pi)+floor(curPos/(2*pi))*2*pi;
            if abs(pos + 2*pi-curPos) < abs(pos-curPos)
                pos = pos + 2*pi;
            end
            posDiff = pos-curPos;
            start = tic;
            ramp = 0;
            %Command the position until the error is satisfied
            while ~robot.checkPosAgainstGiven(robot.fbk,pos)
                ramp = ramp + .02;
                if ramp >= 1; ramp = 1; end
                robot.cmd.position = (curPos + posDiff*ramp).*...
                    robot.directionFlip;
                robot.group.set(robot.cmd);
                pause(robot.pauseTime);
                robot.fbk = robot.group.getNextFeedback();
                %Break out of infinite loops caused by unresponsive modules
                if(toc(start) > 3); 
                    break;
                    %error('Unresponsive Module Error');
                end
            end
        end
                
        %Moves the legs of the robot through given trajectory points
        function followLegTraj(robot,trajPoints,startPt,endPt)
            %Adjust the given trajectory to be within +1 rotation of the
            %current position
            robot.fbk = robot.group.getNextFeedback();
            curPos = robot.fbk.position'.*robot.directionFlip';
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
                robot.cmd.position = pos'.*robot.directionFlip;
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
            curPos = robot.fbk.position'.*robot.directionFlip';
            goalPos = 2*pi*ceil(curPos/(2*pi));
            %Generate and execute a smooth trajectory
            standUpTraj = robot.generateLegTraj([curPos,goalPos],[0,1]);
            robot.followLegTraj(standUpTraj,1,size(standUpTraj,2));
        end
        
        function standUpReverse(robot) 
            %Robot stands up upside-down
            
            %Find current position and next possible position
            robot.fbk = robot.group.getNextFeedback();
            curPos = robot.fbk.position'.*robot.directionFlip';
            goalPos = 2*pi*ceil(curPos/(2*pi))-pi;
            
            %Generate and execute a smooth trajectory
            standUpTraj = robot.generateLegTraj([curPos,goalPos],[0,1]);
            robot.followLegTraj(standUpTraj,1,size(standUpTraj,2));
        end
        
        %Moves the robot forward by one step using the selected gait
        function takeStep(robot,stepSize,stepTime,gaitName, upsideDown)
            switch gaitName
                case 'tripod'
                    robot.takeStepTripod(stepSize,stepTime,upsideDown);
                case 'wave'
                    %prevents it from pulling both values and shorting out
                    robot.takeStepWave(stepSize,stepTime);
            end
        end
        
        %Moves the robot backwards one step using the tripod gait
        function takeStepBackwards(robot,stepSize,stepTime,upsideDown)
            %Generate the waypoints with timesteps for one step
            robot.fbk = robot.group.getNextFeedback();
            curPos = robot.fbk.position'.*robot.directionFlip';
            
            if(upsideDown == true)           
            %Change step directions to allow for upside-down walking
            %This section of the code may break if the stepSize matrix 
            %changes size.
          	stepSize1 = stepSize(1);
            stepSize2 = stepSize(2);
            
            stepSize(1) = stepSize2;
            stepSize(2) = stepSize1;
            
            pos1 = 2*pi*round(curPos/(2*pi)) + ...
                [stepSize(1); 2*pi-stepSize(1); stepSize(1);...
                2*pi-stepSize(2); stepSize(2); 2*pi-stepSize(2)];
            pos2 = pos1 + [2*pi-2*stepSize(1); 2*stepSize(1); ...
                2*pi-2*stepSize(1); 2*stepSize(2); 2*pi-2*stepSize(2);...
                2*stepSize(2)];
            
            %This makes the robot actaully go upside-down.
            pos1 = pos1+ pi*[1 -1 1 -1 1 -1]';
            pos2 = pos2+ pi*[1 -1 1 -1 1 -1]';
            
            else
            %Robot is not upside-down (normal step)
            pos1 = 2*pi*round(curPos/(2*pi)) - ...
                [stepSize(1); 2*pi-stepSize(1); stepSize(1);...
                2*pi-stepSize(2); stepSize(2); 2*pi-stepSize(2)];
            pos2 = pos1 - [2*pi-2*stepSize(1); 2*stepSize(1); ...
                2*pi-2*stepSize(1); 2*stepSize(2); 2*pi-2*stepSize(2);...
                2*stepSize(2)];
            end
            
            stepPoints = [curPos,pos1,pos2];
            stepTimes = linspace(0,stepTime,size(stepPoints,2));
            speeds = zeros(6,3);
            %Generate and execute the trajectory
            walkTraj = robot.generateLegTraj(stepPoints,stepTimes,speeds);
            robot.followLegTraj(walkTraj,1,size(walkTraj,2));
        end
        
        %Moves the robot forward by one step using the tripod gait
        function takeStepTripod(robot,stepSize,stepTime, upsideDown)
            %Generate the waypoints with timesteps for one step
            robot.fbk = robot.group.getNextFeedback();
            curPos = robot.fbk.position'.*robot.directionFlip';
          
            if (upsideDown == true)
            %Change step directions to allow for upside-down walking
            %This section of the code may break if the stepSize matrix 
            %changes size.
            stepSize1 = stepSize(1);
            stepSize2 = stepSize(2);
            
            stepSize(1) = stepSize2;
            stepSize(2) = stepSize1;
            
            pos1 = 2*pi*round(curPos/(2*pi)) - ...
                [stepSize(1); 2*pi-stepSize(1); stepSize(1);...
                2*pi-stepSize(2); stepSize(2); 2*pi-stepSize(2)];
            pos2 = pos1 - [2*pi-2*stepSize(1); 2*stepSize(1); ...
                2*pi-2*stepSize(1); 2*stepSize(2); 2*pi-2*stepSize(2);...
                2*stepSize(2)];
            %negatives added to create backward walking
            
            %Makes the robot go upside-down
                pos1 = pos1+pi*[-1 1 -1 1 -1 1]';
                pos2 = pos2+pi*[-1 1 -1 1 -1 1]'; 
                
            else
            %Robot is not upside-down (normal step)
            pos1 = 2*pi*round(curPos/(2*pi)) + ...
                [stepSize(1); 2*pi-stepSize(1); stepSize(1);...
                2*pi-stepSize(2); stepSize(2); 2*pi-stepSize(2)];
            pos2 = pos1 + [2*pi-2*stepSize(1); 2*stepSize(1); ...
                2*pi-2*stepSize(1); 2*stepSize(2); 2*pi-2*stepSize(2);...
                2*stepSize(2)];
            %no upsideDown part
            end
            
            stepPoints = [curPos,pos1,pos2];
            stepTimes = linspace(0,stepTime,size(stepPoints,2));
            speeds = zeros(6,3);
            
            %Generate and execute the trajectory
            walkTraj = robot.generateLegTraj(stepPoints,stepTimes,speeds);
            robot.followLegTraj(walkTraj,1,size(walkTraj,2));
        end

        
        function flipRobot(robot)
            %This function pushes the robot onto a wall and flips. The
            %entire flipping motion is a trajectory, which makes this
            %function harder to use. Proceed with caution.
            
            %Flips from right-side up to upside-down.
            
            stepTime = 7;
            %This value determines the time it takes the robot to flip.
            %7 is a good value (though 3-8 should work).
            
           %Note: The robot.moveLegsToPos(X') comments allow the coder to
           %test each individual section of the code if any changes are
           %needed. Just comment out the trajectory code and uncomment.
           
           %Creates the robot flipping trajectory.
            robot.fbk = robot.group.getNextFeedback();
            curPos = robot.fbk.position'.*robot.directionFlip';
            
            pos1 = [pi/2;0; 0;0; 0; pi/2]; 
              %robot.moveLegsToPos(pos1');
            
           pos2 = pos1 + [0; pi/2; 0; 0; pi/2; 0];  
           %robot.moveLegsToPos(pos2');   pos1 = mod(pos1,2*pi)+floor(curPos/(2*pi))*2*pi;
       
           
           pos3 = pos2 + [0; 0; pi/2; pi/2; 0; 0];  
           %robot.moveLegsToPos(pos3');
           
           pos4 = pos3 + [0; 0; 0; 0; 0; 0];  
           %robot.moveLegsToPos(pos4');
           
           pos5 = pos4 + [pi; 0; 0; 0; 0; pi];
           %robot.moveLegsToPos(pos5');
           
           pos6 = pos5+[0; pi; 0; 0; pi; 0]; 
           %robot.moveLegsToPos(pos6');
          
           pos7 = pos6+[0; pi/4; 0; 0; pi/4; 0];
           %robot.moveLegsToPos(pos7');

           pos8 = pos7+[0; pi/2; 0; 0; pi/2; 0]; 
           %robot.moveLegsToPos(pos8');
           
           pos9 = pos8+[pi/2; 0; 0; 0; 0; pi/2]; 
           %robot.moveLegsToPos(pos9');
           
           pos10 = pos9+[7*pi/4; 0; 0; 0; 0; 7*pi/4]; 
           %robot.moveLegsToPos(pos10');

       stepPoints = [pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8, pos9, pos10];
       stepTimes = linspace(0,stepTime,size(stepPoints,2));
       speeds = zeros(6,10);
                
       %Generate and execute the trajectory
       walkTraj = robot.generateLegTraj(stepPoints,stepTimes,speeds);
       robot.followLegTraj(walkTraj,1,size(walkTraj,2));
            
        disp('Done Flipping: Remember to Switch to Upside-Down Mode');
        end


        %Moves the robot forward by one step using the tripod gait
        function takeStepWave(robot,stepSize,stepTime)
            %Generate the waypoints with timesteps for one step
            robot.fbk = robot.group.getNextFeedback();
            curPos = robot.fbk.position'.*robot.directionFlip';
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

        function initializeStairs(robot)

           
        end
        
        function takeStepStairs(robot, stepSize, stepTime)
            robot.takeStepTripod([pi/4  pi/4],stepTime, false);
            disp('press enter1');
            pause();
            robot.takeStepTripod([5*pi/24 0],stepTime, false);
             disp('press enter2');
            pause();
            robot.takeStepTripod([pi/4 pi/4],stepTime, false);
             disp('press enter3');
            pause();
           robot.takeStepTripod([5*pi/24 0],stepTime, false);
             disp('press enter4');
            pause();
            robot.takeStepTripod([5*pi/24 0],stepTime, false);
             disp('press enter5');
            pause();
            
            robot.takeStepTripod([pi/4  pi/4],stepTime, false);
            disp('press enter6');
            pause();
            robot.takeStepTripod([0 5*pi/24],stepTime, false);
             disp('press enter7');
            pause();


            
           disp('finished with takeStepStairs');
        end
        
        %Sends a square wave to the motors to dynamically run
        function dynamicRun(robot,stepSize,speed)
            %Speed is a decimal 0-1
            %Calculate velocities needed for step
            maxVel = 9*speed;
            stepTime = (2*pi-2*stepSize)/abs(maxVel);
            minVel = 2*stepSize/stepTime*sign(speed);
            robot.fbk = robot.group.getNextFeedback();
            positions = mod(robot.fbk.position.*robot.directionFlip,2*pi);
            onGround = (positions <= stepSize) + (positions >= 2*pi-stepSize);
            %Proportional control for error adjustment within leg groups
%             k_p = 1;
%             tripod1Pos = sum(positions([1,3,5]))/3;
%             if((tripod1Pos > stepSize) && (tripod1Pos < 2*pi-stepSize))
%                 error = min([tripod1Pos*ones(1,3) - positions([1,3,5]); ...
%                     2*pi+tripod1Pos*ones(1,3) - positions([1,3,5])]);
%                 vel([1,3,5]) = vel([1,3,5])-k_p*error;
%                 %stepPercent = (tripod1Pos-stepSize)/(2*pi-2*stepSize);
%             end
%             tripod2Pos = sum(positions([2,4,6]))/3;
%             if((tripod2Pos > stepSize) && (tripod2Pos < 2*pi-stepSize))
%                 error = min([tripod2Pos*ones(1,3) - positions([2,4,6]); ...
%                     2*pi+tripod2Pos*ones(1,3) - positions([2,4,6])]);
%                 vel([2,4,6]) = vel([2,4,6])-k_p*error;
%                 %stepPercent = (tripod1Pos-stepSize)/(2*pi-2*stepSize);
%             end
            vel = onGround*minVel + ~onGround*maxVel;
            %Set position commands to null for full velocity control
            robot.cmd.position = [];
            %Command first velocity vector
            robot.cmd.velocity = vel.*robot.directionFlip;
            robot.group.set(robot.cmd);
        end
        
        %Makes the robot crouch and jump forward using all 6 legs
        %(EXPERIMENTAL)
        function forwardLeap(robot)
            %Get the robot standing upright
            robot.standUp();
            %Crouch down to prepare for jump
            robot.fbk = robot.group.getNextFeedback();
            curPos = robot.fbk.position.*robot.directionFlip;
            pos = ones(1,6)*3*pi/2 + [pi/4 pi/8 0 0 pi/8 pi/4];
            pos = mod(pos,2*pi)+floor(curPos/(2*pi))*2*pi;
            pos = pos + (curPos-robot.maxErr > pos)*2*pi-2*pi;
            posDiff = pos-curPos;
            start = tic;
            ramp = 0;
            %Command the position until the error is satisfied
            while ~robot.checkPosAgainstGiven(robot.fbk,pos)
                ramp = ramp + .02;
                if ramp >= 1; ramp = 1; end
                robot.cmd.position = (curPos + posDiff*ramp).*...
                    robot.directionFlip;
                robot.group.set(robot.cmd);
                pause(robot.pauseTime);
                robot.fbk = robot.group.getNextFeedback();
                %Break out of infinite loops caused by unresponsive modules
                if(toc(start) > 3); 
                    break;
                    %error('Unresponsive Module Error');
                end
            end
            %Set max torque, no position control to jump
            robot.cmd.position = [];
            gains = robot.group.getGains();
            robot.cmd.torque = robot.directionFlip*gains.torqueMaxTarget(1);
            startTime = tic;
            while toc(startTime) < .3
                robot.group.set(robot.cmd);
                pause(robot.pauseTime);
            end
            %Stop the jump and hold position
            robot.fbk = robot.group.getNextFeedback();
            robot.cmd.torque = [];
            robot.cmd.position = robot.fbk.position;
            robot.group.set(robot.cmd);
        end
        
        %Makes the robot crouch and jump upward using all 6 legs
        %(EXPERIMENTAL)
        function upwardLeap(robot)
            %Crouch down to prepare for jump
            robot.fbk = robot.group.getNextFeedback();
            curPos = robot.fbk.position.*robot.directionFlip;
            pos = [pi/2 pi pi pi pi pi/2];
            pos = mod(pos,2*pi)+floor(curPos/(2*pi))*2*pi;
            pos = pos + (curPos-robot.maxErr > pos)*2*pi-2*pi;
            posDiff = pos-curPos;
            start = tic;
            ramp = 0;
            %Command the position until the error is satisfied
            while ~robot.checkPosAgainstGiven(robot.fbk,pos)
                ramp = ramp + .02;
                if ramp >= 1; ramp = 1; end
                robot.cmd.position = (curPos + posDiff*ramp).*...
                    robot.directionFlip;
                robot.group.set(robot.cmd);
                pause(robot.pauseTime);
                robot.fbk = robot.group.getNextFeedback();
                %Break out of infinite loops caused by unresponsive modules
                if(toc(start) > 3) 
                    break;
                    %error('Unresponsive Module Error');
                end
            end
            %Set max torque, no position control to jump
            robot.cmd.position = [];
            gains = robot.group.getGains();
            robot.cmd.torque = robot.directionFlip*gains.torqueMaxTarget(1);
            %Dont move the front legs for this one
            robot.cmd.torque(3) = 0;
            robot.cmd.torque(4) = 0;
            startTime = tic;
            while toc(startTime) < .65
                robot.group.set(robot.cmd);
                pause(robot.pauseTime);
            end
            %Stop the jump and hold position
            robot.fbk = robot.group.getNextFeedback();
            robot.cmd.torque = [];
            robot.cmd.position = robot.fbk.position;
            robot.group.set(robot.cmd);
        end
    end
end