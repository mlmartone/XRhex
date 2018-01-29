function [legPos] = bodyAngleToLegAngles(bodyTh)
% Solves the geometry to tilt the body of RHex by changing home leg angles

%Magic numbers that describe RHex body shape FIX LATER
legRad = 3; %inches
minDist = 2; %inches
legSpread = 14.5; %inches
%Check if body angle is possible
if(abs(bodyTh) > atan2(legRad*2-minDist,legSpread))
    error('Not admissible body angle for current RHex configuration.');
end
%Calculate lower leg distance needed for correct angle (assuming upper leg
%is max height)
dLow = 2*legRad - sin(abs(bodyTh))*legSpread;
%Calculate angle offset for lower leg
lowerLegTh = acos((dLow-3)/3) + abs(bodyTh);
%Calculate middle leg distance needed for correct angle (assuming upper leg
%is max height)
dMid = 2*legRad - sin(abs(bodyTh))*legSpread/2;
%Calculate angle offset for lower leg
middleLegTh = acos((dMid-3)/3) + abs(bodyTh);
upperLegTh = abs(bodyTh);
%Assign angles to legs depending on sign of angle
if(sign(bodyTh) == 1)
    legPos = [lowerLegTh middleLegTh upperLegTh upperLegTh middleLegTh ...
        lowerLegTh];
else
    legPos = [upperLegTh middleLegTh lowerLegTh lowerLegTh middleLegTh ...
        upperLegTh];
end
end

