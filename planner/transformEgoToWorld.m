function worldXY = transformEgoToWorld(egoXY, egoPose)
%TRANSFORMEGOTOWORLD  Rotate+translate ego-frame (ISO 8855) points into world frame.
%   egoXY   : [N x 2] points in ego frame (x forward, y left)
%   egoPose : [x_ego, y_ego, heading_ego] in world frame
%   worldXY : [N x 2] points in world frame

c = cos(egoPose(3));
s = sin(egoPose(3));
R = [c -s; s c];
worldXY = (R * egoXY')' + egoPose(1:2);
end