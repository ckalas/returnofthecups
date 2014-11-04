format long;

L1 = 94;
L2 = 165;
theta1 = 90; % degrees
theta2 = asin((-L1*sind(theta1))/L2) * 180 / pi

horizontal_reach = L1*cosd(theta1) + L2*cosd(theta1 + theta2)
vertical_read = L1*sind(theta1) + L2*sind(theta1 + theta2)
