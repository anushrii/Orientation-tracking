function [ a_x , a_y, a_z] = dcm2angle(R)

a_x = atan2(R(2,3),R(3,3));

a_y = asin(R(1,3));
a_z = atan2(R(1,2),R(1,1));
