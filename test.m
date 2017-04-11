%Get joystick handle
joy = vrjoystick(1);
stickDeadZone = .08;
while true
    [axes, buttons, povs] = read(joy);
    pause(.01);
    povs
%     plot(axes(1),-axes(2),'g*');
%     hold on;
%     axis([-1 1 -1 1]);
%     plot(axes(3),-axes(4),'r*');
%     legend('Left Stick','Right Stick');
%     hold off;
end