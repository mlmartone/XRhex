%% XRhex Setup
%Clean up the workspace
clear;
%Setup module groups and hebi functions, then define the robot object
family = 'X8-3';
names = {'X-80188','X-80096','X-80182','X-80091','X-80093','X-80095'};
if(~exist('setupComplete','var'))
    [setupComplete,group] = setupRHex(family,names);
    robot = XRhex(group);
end
group.startLog('Directory','logs');
    pause(3);
    log = group.stopLog();
    gyroOffsets = [mean(log.gyroX); mean(log.gyroY); mean(log.gyroZ)];
    accelOffsets = [mean(log.accelX); mean(log.accelY)+9.81; mean(log.accelZ)];
    save offsets gyroOffsets accelOffsets
    display('Gyros and Accelerometers calibrated!');
