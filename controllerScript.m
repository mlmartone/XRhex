%%% XRhex Controller Script %%%
%This code sets up controller support for the original XRhex robot and
%allows a user to drive the robot
%Created 4/9/17 - Updated 4/11/17
%  X-Rhex Layout  %
%      Front      %
%  -3---------4-  %
% | |         | | %
%   |         |   %
%  -2         5-  %
% | |         | | %
%   |         |   %
%  -1---------6-  %
% |      ^      | %

%% XRhex Setup
%Clean up the workspace
clear;
%Setup module groups and hebi functions, then define the robot object
family = 'X8-3';
names = {'X-80188','X-80096','X-80182','X-80091','X-80093','X-80095'};
%family = 'X5-1';
%names = {'X-00131','X-00147','X-00111','X-00033','X-00104','X-00129'};
if(~exist('setupComplete','var'))
    [setupComplete,group] = setup(family,names);
    robot = XRhex(group);
end
minStepTime = 2;
maxStepTime = 10;
minStepSize = 0;
maxStepSize = pi/3;

%% Joystick Setup
%Get joystick handle
joy = vrjoystick(1);
stickDeadZone = .08;
buttonIgnore = .5; %s
lastModeChange = tic;
lastSizeChange = tic;
lastTimeChange = tic;
printInstructions();

%% Stance Setup
%Move legs to upward position
robot.moveLegsToPos(ones(1,6)*pi);
%Set default walking parameters
gait = 'tripod';
stepSize = pi/6;
stepTime = 3;
cmd = CommandStruct();

%% Control Loop
while true
    %Read in the next input from the joystick
    [sticks, buttons, dpad] = read(joy);
    sticks = sticks.*[1 -1 1 -1];
    
    if dpad == 0; robot.takeStep([stepSize stepSize],stepTime,gait); end
    if dpad == 45;
        robot.takeStep([stepSize*sqrt(2) stepSize/sqrt(2)],stepTime,gait);
    end
    if dpad == 90;
        robot.takeStep([maxStepSize minStepSize],stepTime,gait);
    end
    if dpad == 135;
        robot.takeStepBackwards([stepSize*sqrt(2) stepSize/sqrt(2)],stepTime);
    end
    if dpad == 180;
        robot.takeStepBackwards([stepSize stepSize],stepTime);
    end
    if dpad == 225;
        robot.takeStepBackwards([stepSize/sqrt(2) stepSize*sqrt(2)],stepTime);
    end
    if dpad == 270;
        robot.takeStep([minStepSize maxStepSize],stepTime,gait);
    end
    if dpad == 315;
        robot.takeStep([stepSize/sqrt(2) stepSize*sqrt(2)],stepTime,gait);
    end
    
    if buttons(1); robot.moveLegsToPos(ones(1,6)*pi/2); end
    if buttons(2); robot.moveLegsToPos(ones(1,6)*2*pi); end
    if buttons(3); robot.moveLegsToPos(ones(1,6)*3*pi/2); end
    if buttons(4); robot.moveLegsToPos(ones(1,6)*pi); end
    if buttons(5) && toc(lastTimeChange) > buttonIgnore;
        lastTimeChange = tic;
        stepTime = stepTime - .25;
        if stepTime <= minStepTime
            stepTime = minStepTime;
        end
        disp(sprintf('Step time = %2.2f s',stepTime));
    end
    if buttons(6) && toc(lastTimeChange) > buttonIgnore;
        lastTimeChange = tic;
        stepTime = stepTime + .25;
        if stepTime >= maxStepTime
            stepTime = maxStepTime;
        end
        disp(sprintf('Step time = %2.2f s',stepTime));
    end
    if buttons(7) && toc(lastSizeChange) > buttonIgnore;
        lastSizeChange = tic;
        stepSize = stepSize - pi/12;
        if stepSize <= minStepSize
            stepSize = minStepSize;
        end
        disp(sprintf('Step size = %f rad',stepSize));
    end
    if buttons(8) && toc(lastSizeChange) > buttonIgnore;
        lastSizeChange = tic;
        stepSize = stepSize + pi/12;
        if stepSize >= maxStepSize
            stepSize = maxStepSize;
        end
        disp(sprintf('Step size = %f rad',stepSize));
    end
    if buttons(9) && toc(lastModeChange) > buttonIgnore;
        lastModeChange = tic;
        if strcmp(gait,'tripod')
            gait = 'wave';
        elseif strcmp(gait,'wave')
            gait = 'tripod';
        end
        disp(sprintf('Selected Gait = %s',gait));
    end
    if buttons(10); break; end
    if buttons(11); printInstructions(); end
    if buttons(12); robot.standUp(); end
    if sum(buttons) == 0; robot.group.set(robot.cmd); end
    pause(robot.pauseTime)
end