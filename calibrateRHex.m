function [gyroOffsets] = calibrateRHex(group)
    group.startLog('Directory','logs');
    pause(3);
    log = group.stopLog();
    gyroOffsets = [mean(log.gyroX); mean(log.gyroY); mean(log.gyroZ)];
    accelOffsets = [mean(log.accelX); mean(log.accelY)+9.81; mean(log.accelZ)];
    save offsets gyroOffsets accelOffsets
    display('Gyros and Accelerometers calibrated!');
end