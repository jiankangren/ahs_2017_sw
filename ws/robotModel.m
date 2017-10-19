% function xdot = robotModel(x, v1, v2 )
% %ROBOTMODEL Summary of this function goes here
% %   Detailed explanation goes here
% xdot = zeros(4,1);
% l = 0.5;
% xdot(1) = cos(x(3))*cos(x(4))*v1;
% xdot(2) = sin(x(3))*cos(x(4))*v2;
% xdot(3) = sin(x(4)) * v1 /l;
% xdot(4) = v2;
% end
% 
function xdot = robotModel(x, v, phi)
%ROBOTMODEL Summary of this function goes here
%   Detailed explanation goes here
xdot = zeros(3,1);
l =1;
xdot(1) = cos(x(3)) * v;
xdot(2) = sin(x(3)) * v;
xdot(3) = tan(phi) * v/l;
xdot(4) = phi;
end


