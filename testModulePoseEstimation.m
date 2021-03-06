%% Clean up the workspace
clear; close all; clc;
%% Setup module groups and hebi functions
family = 'X8-3';
names = {'X-80171'};%,'X-80096','X-80182','X-80091','X-80093','X-80095'};
addpath('hebi');
currentDir = fileparts(mfilename('fullpath'));
addpath(fullfile(currentDir,'hebi'));
group = HebiLookup.newGroupFromNames(family,names);
%% Setup pose filter and plot setup
%Continuously find pose of a module and plot axes
poseFilter = HebiPoseFilter();
%poseFilter.setYaw(pi);
%Make axes representation
xAxis = [1 0 0 0]';
yAxis = [0 1 0 0]';
zAxis = [0 0 1 0]';
%Make RHex representation
body = [8 0 6 0;8 0 -6 0;0 0 -6 0;-8 0 -6 0;-8 0 6 0;0 0 6 0;8 0 6 0];
%Plot setup
figure();
axisSize = 10;
axis([-1 1 -1 1 -1 1]*axisSize);
%% Continuously find pose of a module and plot axes
while true
    %Update filter
    fbk = group.getNextFeedback();
    accels = [fbk.accelX, fbk.accelY, fbk.accelZ];
    gyros = [fbk.gyroX, fbk.gyroY, fbk.gyroZ];
    poseFilter.update(accels, gyros, fbk.time);
    pose = poseFilter.getPose();
    %Plotting
    clf;
    title('Module Pose Tracking');
    hold on;
    axis([-1 1 -1 1 -1 1]*axisSize);
    view(45,45);
    %Plot base axes
    plot3([0,1],[0,0],[0,0],'k');
    plot3([0,0],[0,1],[0,0],'k');
    plot3([0,0],[0,0],[0,1],'k');
    %Get new axes locations
    xNew = pose*xAxis;
    yNew = pose*yAxis;
    zNew = pose*zAxis;
    %Reallign the body to be drawn
    newBody = (pose*body')';
    plot3(newBody(:,1),newBody(:,2),newBody(:,3),'k','LineWidth',3);
    scatter3(newBody(:,1),newBody(:,2),newBody(:,3),150,'MarkerEdgeColor',...
        'k','MarkerFaceColor','r');
    plot3([0,xNew(1)],[0,xNew(2)],[0,xNew(3)],'r');
    plot3([0,yNew(1)],[0,yNew(2)],[0,yNew(3)],'g');
    plot3([0,zNew(1)],[0,zNew(2)],[0,zNew(3)],'b');
    %Pause for plotting
    pause(.01);
end