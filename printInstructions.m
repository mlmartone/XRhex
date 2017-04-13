%Displays control instructions for X-Rhex
%Created 4/12/17 - Updated 4/13/17
function printInstructions()
fprintf('\n \n \n');
disp('Driver Controls for X-Rhex 1.0:');
disp('-------------------------------');
disp('Button 1: Move all legs to back position (pi/2 rad)');
disp('Button 2: Move all legs to up position (pi rad)');
disp('Button 3: Move all legs to forward position (3*pi/2 rad)');
disp('Button 4: Move all legs to down position (0 rad)');
disp('Button 5: Decrease step time');
disp('Button 6: Increase step time');
disp('Button 7: Decrease step size');
disp('Button 8: Increase step size');
disp('Button 9: Toggle selected gait');
disp('Button 10: Quit');
disp('Button 11: Display Instructions');
disp('Button 12: Unmapped');
disp('D-pad: Drive robot forward, arc turn, or point turn');
disp('Stick 1: Unmapped');
disp('Stick 2: Unmapped');
end