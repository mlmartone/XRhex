%%% XRhex Controller Script %%%
%This code sets up controller support for the original XRhex robot and
%allows a user to drive the robot
%Created 4/9/17 - Updated 1/18/18
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
clear; close all; clc;
%Setup module groups and hebi functions, then define the robot object
family = 'X8-3';
names = {'X-80188','X-80096','X-80182','X-80091','X-80093','X-80095'};
if(~exist('setupComplete','var'))
    [setupComplete,group] = setupRHex(family,names);
    robot = XRhex(group);
end
minStepTime = 2;
maxStepTime = 10;
minStepSize = 0;
maxStepSize = pi/3;
upsideDown = false;

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
    
    if buttons(1); robot.moveLegsToPos([1,-1,1,-1,1,-1]*stepSize); end
    if abs(sticks(2)) > 3*stickDeadZone;
        robot.dynamicRun(stepSize,sticks(2));
    else
        robot.group.set(CommandStruct());
    end
    %Quit loop on button press
    if buttons(10); 
        robot.group.set(CommandStruct());
        break; 
    end
    
    %if sum(buttons) == 0; robot.group.set(robot.cmd); end
    pause(robot.pauseTime)
end