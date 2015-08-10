%conversion of raw IMU data to physical units:
function [A , W, Ra, Rw , Euler_A, Euler_W] =  setright_IMU(vals)
%load  ('C:\Users\sabhajit singh\Desktop\learning in robotics\project2\imu\imuRaw1.mat')
Ax = vals(1,:);
Ay = vals(2,:);
Az = vals(3,:);
Aa = [Ax; Ay; Az];

% for accelerometers
bias_Ax = 511;% mean(Ax(1:400));
bias_Ay = 511;% mean(Ay(1:400));
bias_Az = 511;% mean(Az(1:400));
bias = [ bias_Ax; bias_Ay; bias_Az]; 

% sensitivity for accelerometer
 sa = 300; Vref = 3300; 
 sfa = Vref/(1023*sa); % scale factor for acc
%sfa=1/300;
Ab = bsxfun(@minus,Aa,bias);
Ab(1:2,:) = -Ab(1:2,:);
Ar = Ab*sfa;

tmp=sqrt(sum(Ar.*Ar));
Ar=bsxfun(@rdivide,Ar,tmp);
A = Ar;

%for conversion to quaternions
yaw =zeros(1,size(Ar,2));
pitch = atan2(-Ar(1,:),sqrt(Ar(2,:).*Ar(2,:) + Ar(1,:).*Ar(1,:)));
roll = atan2(-Ar(2,:),Ar(3,:));
Ra =  angle2dcm(roll,pitch,yaw);
qua = dcm2q(Ra);
Euler_A  = quat2euler(qua);





% for gyro, finding angualar velocity
Wx = vals(4,:);
Wy = vals(5,:);
Wz = vals(6,:);
W = [ Wx; Wy; Wz];

% subtracting bias
bias_Wx = mean(Wx(1:400)); 
bias_Wy = mean(Wy(1:400));
bias_Wz = mean(Wz(1:400));
bias_W = [ bias_Wx; bias_Wy; bias_Wz];

Wr = bsxfun(@minus,W,bias_W);

% scaling it to physical values
sw = 4.1;
Vref = 3300;
sfw = (Vref/1023)/sw;
Wr = Wr*sfw*pi/180;
%%in the g frame first component is sign inverted
W=[Wr(2:3,:); -Wr(1,:)];
%W = Wr;
%for axis angle representation
% given angualr velocity multiplying by delta_t will give 
% differential rotation i.e angle

delta_t = 1/100;
norm_W  = sqrt(sum(Wr.*Wr));
alpha_delta = norm_W*delta_t;%angle
e_delta = bsxfun(@rdivide,Wr,norm_W);

%corresponding quaternion_delta

quat_delta = [cos(alpha_delta/2) ;bsxfun(@times,e_delta,sin(alpha_delta/2))];
quat_delta = quat_delta';
quat_i = [1 0 0 0 ];
% new state q_k+1 = q_k*delta_q,
% used a loop to calculate the states.
% starting with initial state of quaternion  [1 0 0 0]


 q_out = zeros(size(W,2),4);
 for i=1:size(W,2)
     
     q_out(i,:) = quatmultiply(quat_i,quat_delta(i,:));
     q_out(i,:) = quat_normalize(q_out(i,:));
     quat_i = q_out(i,:);
 end
 Rw =  quat2dcm(q_out);
 
 Euler_W = quat2euler(q_out);


 
 








