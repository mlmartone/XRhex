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
d = legRad - tan(abs(bodyTh))*legSpread;
%Calculate angle offset for lower leg
alpha = asin(d/legRad);
%Final calculation of leg angles
lowerLegTh = abs(bodyTh) + alpha;
upperLegTh = abs(bodyTh);
middleLegTh = (upperLegTh+lowerLegTh)/2;
%Assign angles to legs depending on sign of angle
if(sign(bodyTh) == 1)
    legPos = [lowerLegTh middleLegTh upperLegTh upperLegTh middleLegTh ...
        lowerLegTh];
else
    legPos = [upperLegTh middleLegTh lowerLegTh lowerLegTh middleLegTh ...
        upperLegTh];
end

end

