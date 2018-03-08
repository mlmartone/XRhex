figure();
axis([-1 1 -1 1]);
hold on;
while true
    [sticks, buttons, dpad] = read(joy);
    sticks = sticks.*[1 -1 1 -1];
    plot(sticks(1),sticks(2),'r*');
    plot(sticks(3),sticks(4),'g*');
    pause(.01);
end