%%% XRhex Walking Script %%%
%This code sets up and runs walking motion for the original XRhex robot
%Created 3/1/17 - Updated 4/8/17
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
family = 'X5-1';
names = {'X-00131','X-00147','X-00111','X-00033','X-00104','X-00129'};
if(~exist('setupComplete','var'))
    [setupComplete,group] = setup(family,names);
    robot = XRhex(group);
end

%% Stance Setup
%Move legs to upward position, then prompt user to make it stand up
robot.moveLegsToPos(ones(1,6)*pi);
disp('Set the robot down on a level surface, then press ENTER when ready.');
pause(1);
%Stand up, walk n steps, return to standing pose, then stop
robot.standUp();
robot.holdPos(1);
n = 10;
for i = 1:1:n
    robot.takeStep(pi/6,3,'walk');
end
robot.moveLegsToPos(ones(1,6)*2*pi);
robot.holdPos(1);