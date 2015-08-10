function q_out =quat_normalize(Qu)



% Find the magnitude of each quaternion
q_norm=sqrt(sum(Qu.^2,2));

q_norm=[q_norm q_norm q_norm q_norm];
% Divide each element of q by appropriate qmag
q_out=Qu./q_norm;

