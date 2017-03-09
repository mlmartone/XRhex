function out = evalError(fbk,maxErr)
    error = abs(fbk.position-fbk.positionCmd);
    out = max(error) < maxErr;
end