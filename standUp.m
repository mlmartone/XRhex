%Set up paths and hebi functions
if(~exist('setupComplete','var'))
    clear; clc; close all;
    addpath('hebi');
    currentDir = fileparts(mfilename('fullpath'));
    addpath(fullfile(currentDir , 'hebi'));
    HebiLookup;
    family = 'X5-1';
    names = {'X-00131','X-00147','X-00111','X-00043','X-00104','X-00129'};
    allModules = HebiLookup.newGroupFromNames(family, names);
    group1 = HebiLookup.newGroupFromNames(family,{'X-00131','X-00111','X-00104'});
    group2 = HebiLookup.newGroupFromNames(family,{'X-00147','X-00043','X-00129'});
    setupComplete = true;
end

%Set constant values and tuning
maxErr = .02; %rad
pauseTime = .05; %s

%Set all legs to the up position, wait for further instruction
disp('Begin Movement');
cmd1 = CommandStruct();
cmd2 = CommandStruct();
pos1 = pi;
pos2 = pos1;
startTime = tic;
fbk1 = group1.getNextFeedback();
fbk2 = group2.getNextFeedback();
while ~evalError(fbk1,maxErr) || ~evalError(fbk2,maxErr)
    cmd1.position = [pos1 pos1 -pos1];
    group1.set(cmd1);
    pause(pauseTime);
    fbk1 = group1.getNextFeedback();
    cmd2.position = [pos2 -pos2 -pos2];
    group2.set(cmd2);
    pause(pauseTime);
    fbk2 = group2.getNextFeedback();
end
disp('Set the robot down on a level surface, then press ENTER when ready.');
pause;

%Stand the robot up slowly
cmd1 = CommandStruct();
cmd2 = CommandStruct();
pos = pi;
startTime = tic;
fbk1 = group1.getNextFeedback();
fbk2 = group2.getNextFeedback();
while pos < 2*pi
    pos = pos + pi/10;
    cmd1.position = [pos pos -pos];
    group1.set(cmd1);
    pause(pauseTime);
    fbk1 = group1.getNextFeedback();
    cmd2.position = [pos -pos -pos];
    group2.set(cmd2);
    pause(pauseTime);
    fbk2 = group2.getNextFeedback();
end
while true
    group1.set(cmd1);
    group2.set(cmd2);
    pause(pauseTime);
end