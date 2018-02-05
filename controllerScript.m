%%% XRhex Controller Script %%%
%This code sets up controller support for the original XRhex robot and
%allows a user to drive the robot
%Created 4/9/17 - Updated 1/18/18
%  X-Rhex Layout  %
%      Front      %
%  -3---------4-  %
% | |         | | %
%   |         |   %
%  -2         5-  %
% | |         | | %
%   |         |   %
%  -1---------6-  %
% |      ^      | %

%% XRhex Setup
%Clean up the workspace
clear;
%Setup module groups and hebi functions, then define the robot object
family = 'X8-3';
names = {'X-80188','X-80096','X-80182','X-80091','X-80093','X-80095'};
if(~exist('setupComplete','var'))
    [setupComplete,group] = setup(family,names);
    robot = XRhex(group);
end
minStepTime = 2;
maxStepTime = 10;
minStepSize = 0;
maxStepSize = pi/3;
upsideDown = false;

%% Joystick Setup
%Get joystick handle
joy = vrjoystick(1);
stickDeadZone = .08;
buttonIgnore = .5; %s
lastModeChange = tic;
lastSizeChange = tic;
lastTimeChange = tic;
printInstructions();

%% Stance Setup
%Move legs to upward position
robot.moveLegsToPos(ones(1,6)*pi);
%Set default walking parameters
gait = 'tripod';
stepSize = pi/6;
stepTime = 3;
cmd = CommandStruct();

%% Control Loop
while true
    %Read in the next input from the joystick
    [sticks, buttons, dpad] = read(joy);
    sticks = sticks.*[1 -1 1 -1];
    %Direciton Pad Control, tell robot to walk forward, backward, turn,
    %etc.
    %Step sizes allow for turning by altering steps on right and left
    %sides, bigger step disparities allow for sharper turns
    if dpad == 0; robot.takeStep([stepSize stepSize],stepTime,gait, upsideDown); end
    if dpad == 45;
        robot.takeStep([stepSize/sqrt(2) stepSize*sqrt(2)],stepTime,gait, upsideDown);
    end
    if dpad == 90;
        robot.takeStep([minStepSize maxStepSize],stepTime,gait,upsideDown);
    end
    if dpad == 135;
        robot.takeStepBackwards([stepSize/sqrt(2) stepSize*sqrt(2)],stepTime, upsideDown);
    end
    if dpad == 180;
        robot.takeStepBackwards([stepSize stepSize],stepTime, upsideDown);
    end
    if dpad == 225;
        robot.takeStepBackwards([stepSize*sqrt(2) stepSize/sqrt(2)],stepTime, upsideDown);
    end
    if dpad == 270;
        robot.takeStep([maxStepSize minStepSize],stepTime,gait, upsideDown);
    end
    if dpad == 315;
        robot.takeStep([stepSize*sqrt(2) stepSize/sqrt(2)],stepTime,gait, upsideDown);
    end
    
    %Move all legs to clock positions, mostly for debug purposes
    if buttons(1); robot.moveLegsToPos(ones(1,6)*pi/2); end
    if buttons(2); robot.moveLegsToPos(ones(1,6)*2*pi); end
    if buttons(3); robot.moveLegsToPos(ones(1,6)*3*pi/2); end
    if buttons(4); robot.moveLegsToPos(ones(1,6)*pi); end
    
    %Alter step parameters, prevent duplicate button presses
    if buttons(5) && toc(lastTimeChange) > buttonIgnore;
        lastTimeChange = tic;
        stepTime = stepTime - .25;
        if stepTime <= minStepTime
            stepTime = minStepTime;
        end
        disp(sprintf('Step time = %2.2f s',stepTime));
    end
    if buttons(6) && toc(lastTimeChange) > buttonIgnore;
        lastTimeChange = tic;
        stepTime = stepTime + .25;
        if stepTime >= maxStepTime
            stepTime = maxStepTime;
        end
        disp(sprintf('Step time = %2.2f s',stepTime));
    end
    if buttons(7) && toc(lastSizeChange) > buttonIgnore;
        lastSizeChange = tic;
        stepSize = stepSize - pi/12;
        if stepSize <= minStepSize
            stepSize = minStepSize;
        end
        disp(sprintf('Step size = %f rad',stepSize));
    end
    if buttons(8) && toc(lastSizeChange) > buttonIgnore;
        lastSizeChange = tic;
        stepSize = stepSize + pi/12;
        if stepSize >= maxStepSize
            stepSize = maxStepSize;
        end
        disp(sprintf('Step size = %f rad',stepSize));
    end
    if buttons(9) && toc(lastModeChange) > buttonIgnore;
        lastModeChange = tic;
        if strcmp(gait,'tripod')
            gait = 'wave';
        elseif strcmp(gait,'wave')
            gait = 'tripod';
        end
        disp(sprintf('Selected Gait = %s',gait));
    end
    %Quit loop on button press
    if buttons(10); break; end
    %Print instructions for user in console
    if buttons(11); 
        printInstructions(); 
        %robot.upwardLeap();
        if(upsideDown == true)
        upsideDown = false;
        disp('Upside Down is');
        disp(upsideDown);
     
        else
        upsideDown = true;
        disp('Upside Down is');
        disp(upsideDown);
         disp('Note: Backward Walking Experimental');
        end
    end
    %Stand up
    if buttons(12); %right joystick
        if(upsideDown == true)
            robot.standUpReverse();   
        else
             robot.standUp();
        end
    end 
    %originally robot.standUp()
    if sum(buttons) == 0; robot.group.set(robot.cmd); end
    pause(robot.pauseTime)
end