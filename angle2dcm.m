
function R = angle2dcm(roll,pitch,yaw)

R = zeros(3,3);
for i = 1:size(roll,2)
rroll_M=[1    0           0;
    0    cos(roll(1,i))   sin(roll(1,i));
    0    -sin(roll(1,i))  cos(roll(1,i))];

pitch_M=[cos(pitch(1,i))      0       -sin(pitch(1,i));
    0               1       0;
    sin(pitch(1,i))      0       cos(pitch(1,i))];

yaw_M= [cos(yaw(1,i))        sin(yaw(1,i))        0;
    -sin(yaw(1,i))       cos(yaw(1,i))        0;
    0               0               1];

R =[ R (yaw_M*pitch_M)*rroll_M];
end
R = R(:,4:end);
R = reshape(R,3,3,[]);

