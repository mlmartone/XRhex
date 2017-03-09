%%% XRhex Walking Script %%%
%This code sets up and runs walking motion for the original XRhex robot
%Created 3/1/17 - Updated 3/2/17
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
    [setupComplete,group1,group2] = setup(family,names);
    robot = XRhex(group1,group2);
end
%Precalculate necessary trajectories
standUpTraj = robot.generateLegTraj([pi,2*pi-pi/6;pi,2*pi+pi/6],[0,1]);
%standUpTraj = robot.generateLegTraj([pi,2*pi;pi,2*pi],[0,1]);
numSteps = 4;
stepBase = [-pi/6 pi/6;...
            pi/6 2*pi-pi/6];
stepTime = .5;
stepTimes = zeros(1,2*numSteps);
for step = 1:1:2*numSteps
    stepPoints(:,2*step-1:2*step) = stepBase+2*pi*step;
    if(step == 1)
        stepTimes(2*step-1) = 0;
        stepTimes(2*step) = stepTime;
    else
        stepTimes(2*step-1) = stepTimes(2*step-2)+stepTime;
        stepTimes(2*step) = stepTimes(2*step-1)+stepTime;
    end
end
walkTraj = robot.generateLegTraj(stepPoints,stepTimes);

%% Stance Setup %%
%Move legs to upward position, then prompt user to make it stand up
robot.moveLegsToPos(pi,pi);
disp('Set the robot down on a level surface, then press ENTER when ready.');
pause;
robot.followLegTraj(standUpTraj,1,size(standUpTraj,2));
robot.holdPos(2);
robot.followLegTraj(walkTraj,1,size(walkTraj,2));
robot.holdPos(10);