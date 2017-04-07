function [setupComplete,group1,group2] = setup(family,names)
    clear; clc; close all;
    addpath('hebi');
    currentDir = fileparts(mfilename('fullpath'));
    addpath(fullfile(currentDir , 'hebi'));
    HebiLookup;
    family = 'X5-1';
    names = {'X-00131','X-00147','X-00111','X-00043','X-00104','X-00129'};
    group2 = HebiLookup.newGroupFromNames(family,{'X-00147','X-00043',...
        'X-00129'});
    group1 = HebiLookup.newGroupFromNames(family,{'X-00131','X-00111',...
        'X-00104'});
    gains1 = group1.getGains();
    gains2 = group2.getGains();
    gains1.positionMaxTarget = ones(1,3)*realmax;
    gains2.positionMaxTarget = ones(1,3)*realmax;
    gains1.positionMinTarget = ones(1,3)*-realmax;
    gains2.positionMinTarget = ones(1,3)*-realmax;
    group1.set('Gains',gains1);
    group2.set('Gains',gains2);
    setupComplete = true;
end