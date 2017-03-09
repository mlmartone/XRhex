function [setupComplete,group1,group2] = setup(family,names)
    clear; clc; close all;
    addpath('hebi');
    currentDir = fileparts(mfilename('fullpath'));
    addpath(fullfile(currentDir , 'hebi'));
    HebiLookup;
    family = 'X5-1';
    names = {'X-00131','X-00147','X-00111','X-00043','X-00104','X-00129'};
    group1 = HebiLookup.newGroupFromNames(family,{'X-00131','X-00111','X-00104'});
    group2 = HebiLookup.newGroupFromNames(family,{'X-00147','X-00043','X-00129'});
    setupComplete = true;
end