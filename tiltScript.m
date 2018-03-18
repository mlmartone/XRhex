%%% XRhex Tilt Script %%%
%This code sets up controller support for the original XRhex robot and
%allows a user to test tilting and pose adjustment
%Created 1/22/18
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
if(~exist('setupComplete','var'))
    [setupComplete,group] = setupRHex(family,names);
    robot = XRhex(group);
end

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
cmd = CommandStruct();

bodyTh = 0;
%% Control Loop
while true
    %Read in the next input from the joystick
    [sticks, buttons, dpad] = read(joy);
    sticks = sticks.*[1 -1 1 -1];
    
    if dpad == 0; end
    if dpad == 45; end
    if dpad == 90; end
    if dpad == 135; end
    if dpad == 180; end
    if dpad == 225; end
    if dpad == 270; end
    if dpad == 315; end
    
    if buttons(1); robot.standUp(); end
    if buttons(2); robot.moveLegsToPos(zeros(1,6)); end
    if buttons(3); end
    if buttons(4); end
    if buttons(5);
        bodyTh = max(bodyTh - .05,-.269);
        robot.biDirMoveLegsToPos(bodyAngleToLegAngles(bodyTh));
    end
    if buttons(6);
        bodyTh = min(bodyTh + .05,.269);
        robot.biDirMoveLegsToPos(bodyAngleToLegAngles(bodyTh));
    end
    if buttons(7); end
    if buttons(8); end
    if buttons(9); end
    if buttons(10); break; end
    if buttons(11); end
    if buttons(12); end
    if sum(buttons) == 0; robot.group.set(robot.cmd); end
    pause(robot.pauseTime)
end