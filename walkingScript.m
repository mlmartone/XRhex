%%% XRhex Walking Script %%%
%This code sets up and runs walking motion for the original XRhex robot
%Created 3/1/17 - Updated 4/7/17
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
clear;
%% Setup %%
%Setup module groups and hebi functions, then define the robot object
family = 'X5-1';
names = {'X-00131','X-00147','X-00111','X-00043','X-00104','X-00129'};
if(~exist('setupComplete','var'))
    %Group1 is legs 1, 3, and 5, Group2 is legs 2, 4, and 6
    [setupComplete,group1,group2] = setup(family,names);
    robot = XRhex(group1,group2);
end

%% Stance Setup %%
%Move legs to upward position, then prompt user to make it stand up
robot.moveLegsToPosGroup(pi,pi);
disp('Set the robot down on a level surface, then press ENTER when ready.');
pause;
%Stand up, walk n steps, return to standing pose, then stop
robot.standUp();
robot.holdPos(2);
n = 10;
for i = 1:1:n
    robot.takeStep(pi/6,2,'walk');
end
robot.moveLegsToPosGroup(0,0);
robot.holdPos(1);