function [x] = motion_command(x, u)
% Updates the robot pose according to the motion model
% x: 3x1 vector representing the robot pose [x; y; theta]
% u: struct containing odometry reading (r1, t, r2).
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

disp("x: "),disp(x)
%TODO: update x according to the motion represented by u
newX = x(1, 1) + u.t*cos(x(3, 1) + u.r1)
newY = x(2, 1) + u.t*sin(x(3, 1) + u.r1)
newTheta = x(3, 1) + u.r1 + u.r2
newTheta = normalize_angle(newTheta)

%TODO: remember to normalize theta by calling normalize_angle for x(3)
x = [newX; newY; newTheta]
end
