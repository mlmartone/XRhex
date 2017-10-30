%%% XRhex Walking Script %%%
%This code sets up and runs walking motion for the original XRhex robot
%Created 3/1/17 - Updated 10/29/17
%     Leg Layout
%       Front      %
%   -3---------4-  %
%  | |         | | %
%    |         |   %
%   -2         5-  %
%  | |         | | %
%    |         |   %
%   -1---------6-  %
%  |      ^      | %

%% Setup
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

%% Stance Setup
%Move legs to upward position, then prompt user to make it stand up
%robot.moveLegsToPos(pi);
disp('Set the robot down on a level surface, then press ENTER when ready.');
pause();
%Stand up, walk n steps, return to standing pose, then stop
%robot.standUp();
n = 10;
robot.cmd = CommandStruct();
robot.cmd.velocity = ones(1,6)*10;
robot.group.set(robot.cmd);
pause();
for i = 1:1:n
    %robot.takeStep([pi/6 pi/6],3,'tripod');
end
%robot.moveLegsToPos(ones(1,6)*2*pi);