%%% XRhex Walking Script Experimential%%%
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

stepSize = [pi/6 , pi/6];
stepTime = 3;

%% Stance Setup
%Move legs to upward position, then prompt user to make it stand up
%robot.moveLegsToPos(pi);
disp('Set the robot down on a level surface, then press ENTER when ready.');
pause();
%Stand up, walk n steps, return to standing pose, then stop
%robot.standUp();
n = 1;
robot.cmd = CommandStruct();
robot.group.set(robot.cmd);

%robot.standUp();
robot.standUpReverse(); %reverse stand up
disp('another pause');
pause();
for i = 1:1:n
    
  % robot.takeStep([pi/6 pi/6],3,'tripod');
     %robot.waveGaitAdriana(pi/6, 3);
  % robot.takeStepTripod([pi/6,pi/6], 3);
  
  %the following code should be takeStepReverseTripod
   robot.fbk = robot.group.getNextFeedback();
   curPos = robot.fbk.position'.*robot.directionFlip';
   
   %added negatives to stepSize
   pos1 = 2*pi*round(curPos/(2*pi)) + ...
         [stepSize(1); -(2*pi-stepSize(1)); stepSize(1); ...
         -(2*pi-stepSize(2)); stepSize(2); -(2*pi-stepSize(2))];
  
   pos2 = pos1 + [-(2*pi-2*stepSize(1)); 2*stepSize(1); ...
         -(2*pi-2*stepSize(1)); 2*stepSize(2); -(2*pi-2*stepSize(2));...
          2*stepSize(2)];
     
  
   stepPoints = [curPos, pos1, pos2];
   stepTimes = linspace(0,stepTime,size(stepPoints,2));
   
   speeds = zeros(6,3); %change 3 to numSteps
            %Generate and execute the trajectory
  walkTraj = robot.generateLegTraj(stepPoints,stepTimes,speeds);
  robot.followLegTraj(walkTraj,1,size(walkTraj,2));
         
     
end
