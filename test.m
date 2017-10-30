%% XRhex Setup
%Clean up the workspace
clear;
%Setup module groups and hebi functions, then define the robot object
family = 'X8-3';
names = {'X-80188'};

addpath('hebi');
currentDir = fileparts(mfilename('fullpath'));
addpath(fullfile(currentDir,'hebi'));
group = HebiLookup.newGroupFromNames(family,names);
%Set the gains so that the modules can turn in one direction for a long
%time, this is a workaround fix, and eventually modules will need to be
%recalibrated or returned to original 0 position
gains = group.getGains();
gains.positionMaxTarget = realmax;
gains.positionMinTarget = -realmax;
group.set('Gains',gains);
%Make command lifetime infinite
group.setCommandLifetime(0);
%Turn off an annoying warning message, this is a workaround fix, and
%eventually should be fixed
warning('off','MATLAB:colon:nonIntegerIndex');
setupComplete = true;
joy = vrjoystick(1);
cmd = CommandStruct();
cmd.position = [0];
group.set(cmd);
