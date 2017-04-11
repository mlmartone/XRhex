%Sets up paths, module groups, and gains to run X-Rhex
function [setupComplete,group] = setup(family,names)
    addpath('hebi');
    currentDir = fileparts(mfilename('fullpath'));
    addpath(fullfile(currentDir,'hebi'));
    group = HebiLookup.newGroupFromNames(family,names);
    %Set the gains so that the modules can turn in one direction for a long
    %time, this is a workaround fix, and eventually modules will need to be
    %recalibrated or returned to original 0 position
    gains = group.getGains();
    gains.positionMaxTarget = ones(1,6)*realmax;
    gains.positionMinTarget = ones(1,6)*-realmax;
    group.set('Gains',gains);
    %Turn off an annoying warning message, this is a workaround fix, and
    %eventually should be fixed
    warning('off','MATLAB:colon:nonIntegerIndex');
    setupComplete = true;
end