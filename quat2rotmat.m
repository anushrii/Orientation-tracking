function [ R ] = quat2rotmat( q )
%QUAT2ROTMAT This function takes in a quaternion q and returns a rotation 
% matrix R 
q0 = q(1); qx=q(2); qy=q(3); qz = q(4);

R = [ (q0^2 + qx^2 - qy^2 - qz^2),        (2*qx*qy -2*q0*qz),          (2*qx*qz + 2*q0*qy);
      (2*q0*qz + 2*qx*qy)     ,     (q0^2 - qx^2 + qy^2 - qz^2),       (2*qy*qz - 2*q0*qx);
      (2*qx*qz - 2*q0*qy)     ,           (2*q0*qx + 2*qy*qz)  ,     (q0^2 - qx^2 - qy^2 + qz^2) ]; 

end