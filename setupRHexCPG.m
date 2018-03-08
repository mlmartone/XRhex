%Sets up the complimentary filter for estimating RHex body pose
function [CF,pose] = setupRHexCPG(group)
    calibrateRHex(group);
    load('offsets.mat');
    CF = RHexComplimentaryFilter(snakeData,'accelOffsets',accelOffsets,...
        'gyroOffsets', gyroOffsets);
    %Get pose reading
    pose = [];
    while isempty(pose)
        CF.update(group.getNextFeedback());
        pose = CF.getBodyPose();
    end
end