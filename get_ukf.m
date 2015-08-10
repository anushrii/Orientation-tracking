function [ Q_q,ukf,bla] = get_ukf(A , W ,ts_i)
%initializations for the Ukf
%covariances


%  P=diag(rand(1,3));
%  Q=diag([0.01 0.06 0.01]);
%  R=diag([1 1 1 ]);
 
 P=diag(rand(1,3));
 Q=diag([0.01 0.06 0.01]);
 R=diag([1 1 1 ]);
 
%  P=diag(0.1*ones(1,3));
% Q=diag(0.2*ones(1,3));
% R=diag([1 1 1 ]);

 blah = 1;
g = [0 0 0 1]; %initial measurement (will be updated later)%%%gravity vector (check for minus plus)
delta_t = 1/50; %( 0.01)
x_hat = [1; 0; 0; 0]; %initial state
ukf=zeros(3,3,size(A,2));

 Q_q=zeros(4,size(A,2));
%%%for quaternion sigma points
for i=1:size(A,2)-1
%     
%     d_t = ts_i(i+1) - ts_i(i);
%     
%     Q = Q + 0.009*d_t;
  %%%for quaternion sigma points
  n = 3; %dimension of the state vector
  try
  L = chol(P + Q); %cholesky decomposition
  catch
      blah = blah +1;
  end
  
  
  
  L1 = sqrt(2*(n-0.5))*L;
  L2 = -L1;
  S = [L1 L2];
  %U = bsxfun(@plus, x_hat, S);
  U = vector_quaternion(S);
  q_i=quatmultiply(x_hat',U');
  
  %%%%%%
%   x_ = quaternion_vector(x_hat);
%   q_i = vector_quaternion(bsxfun(@plus, S, x_));
% to build the process model A(), find delta_q,
%then do quaternion multiplication of present state with delta_q

w = W(:,i); 

norm_w  = sqrt(sum(w.*w));
alpha_delta = norm_w*delta_t;%angle
e_delta = w/norm_w;


%corresponding quaternion_delta

quat_delt = [cos(alpha_delta/2) ;e_delta*sin(alpha_delta/2)];
quat_delta = quat_delt';


qi = quatmultiply(q_i, quat_delta);
qi = quatnormalize(qi);
Y_sigma = qi';

%%%%%gradient descent for mean computation
q = (x_hat)'; % quternion representing previous state qt
iter_no = 0;
error = 1;
while (error > 0.01 && iter_no < 100)
    e_q = quatmultiply(qi,quatinv(q));
     ee = (quaternion_vector(e_q'));
    %barrycentric mean in rotation vector form
    e_mean = mean(quaternion_vector(e_q'),2); 
   
    %switching back to queternion to get qt+1 = e*qt
    e_q = (vector_quaternion(e_mean))';
    q = quatmultiply(e_q,q);
    error = sqrt(sum(e_mean.*e_mean));
    iter_no = iter_no +1;
end

%%computing co-variance of priori state
%convert back to vectors to apply the covariance formula
x_hat_ = q';
% x_bar = quaternion_vector(q');
% Xi = quaternion_vector(Y_sigma);
%Wdash = bsxfun(@minus,Xi,x_ba);
Wdash = ee;
Pdash = cov(Wdash'); % covariance of the transformed set {Yi}


%%%% state update done
%%%%% measurement update starts


%%measurement co-variance



gdash = quatmultiply(Y_sigma',quatmultiply(g,quatinv(Y_sigma')));
 g_vector = quaternion_vector(gdash');
 % comparision of the measurement to the observed value from accelerometer
 Z = g_vector;
 zdash =mean(g_vector,2);
 
 %%%%true readings
 Zi = A(:,i);
 %%% innovation step
 v = Zi - zdash;
 
 Pzz = cov(Z'); 
 Pvv = Pzz + R;
 
 %%%%cross correlation matrix to compute Pxz
 zz = (bsxfun(@minus,Z,zdash))';
 Pxz = S*zz/6;
 
 %%final updating of the equations
 %%kalman gain update
 
    
  K = Pxz*(Pvv^-1);
 
  %posterior update
  %x_hat = x_bar +K*v;
  
  Kk = vector_quaternion(K*v);
  x_hat=(quatmultiply(x_hat_',Kk'))'; 
  bla(:,i) = x_hat;
  P = Pdash - K*Pvv*K';
  %  P = Pdash ;
 
  
Q_q(:,i) = x_hat;
ukf(:,:,i) = quat2dcm((Q_q(:,i))');

end

  