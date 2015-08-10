function [ Qq,ukf ,xhist] = get_ukf_7(A , W )

P=diag(0.0001*ones(1,6));
Q=diag(0.002*ones(1,6));
R=diag(rand(1,6));
xhist=zeros(7,size(A,2));
g = [0 0 0 1]; %initial measurement (will be updated later)%%%gravity vector (check for minus plus)
delta_t = 1/100; %( 0.01)
x_hat = [1; 0 ; 0; 0; 0; 0; 0]; %initial state
ukf=zeros(3,3,size(A,2));

 Qq=zeros(4,size(A,2));
%%%for quaternion sigma points
for i=1:size(A,2)
    
  %%%for quaternion sigma points
  n = 6; %dimension of the state vector
  L = chol(P+Q); %cholesky decomposition
  L1 = sqrt(2*(n-0.5))*L;
  L2 = -L1;
  S = [L1 L2];
  
  X(1:4,:) = (quatmultiply( (x_hat(1:4))',vector_quaternion(S(1:3,:))' ) )';  %%%%checckkkkkkkkkkkkkkkkk
  X(5:7,:) = bsxfun(@plus, S(4:6,:) , x_hat(5:7));
  
  
 % U = bsxfun(@plus, x_hat, S);
 % X = vector_quaternion(U);
  
% to build the process model A(), find delta_q,
%then do quaternion multiplication of present state with delta_q

w = x_hat(5:7); %%NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

norm_w  = sqrt(sum(w.*w));
alpha_delta = norm_w*delta_t;%angle
e_delta = bsxfun(@rdivide,w,norm_w);
e_delta(isnan(e_delta)) = 0;
e_delta(isinf(e_delta)) = 0;


%corresponding quaternion_delta

quat_delt = [cos(alpha_delta/2) ;bsxfun(@times, e_delta,sin(alpha_delta/2))];
quat_delta = quat_delt';

%qint = qmult(x_hat(1:4)', X(1:4,:)');
qi = quatmultiply(X(1:4,:)', quat_delta);  %%%%check!!!!!!!!!!!!!!!!
qi = quatnormalize(qi);
Y(1:4,:) = qi';
Y(5:7,:) =X(5:7,:);

%%%%%gradient descent for mean computation
q = (x_hat(1:4))'; % quternion representing previous state qt
iter_no = 0;
error = 1;
while (error > 0.001 && iter_no < 300)
    e_q = quatmultiply(qi,quatinv(q));
    e_q = quaternion_vector(e_q');
    %barrycentric mean in rotation vector form
    e_mean = mean(e_q,2); 
    %e_vec = quaternion_vector(e_q');
    %switching back to queternion to get qt+1 = e*qt
    e_q = (vector_quaternion(e_mean))';
    q = quatmultiply(e_q,q);
    error = sqrt(sum(e_mean.*e_mean));
    iter_no = iter_no +1;
end
%q =rotqrmean(q');

%%computing co-variance of priori state
%convert back to vectors to apply the covariance formula
x_bar(1:4) = q';%%%%%%%5
x_bar(5:7) = mean(Y(5:7,:),2);%%%%%%%%%%5

%omega = mean(Y(5:7,:),2);
%x_bar = quaternion_vector(q');
%Xi = quaternion_vector(Y_sigma);

%Wdash = bsxfun(@minus,Xi,x_bar);

e_vec  = (quatmultiply(Y(1:4,:)',quatinv(x_bar(1:4))))';
e_vec = quaternion_vector(e_vec);
Wdash = [e_vec; bsxfun(@minus, Y(5:7,:), (x_bar(5:7))')];
Pdash = cov(Wdash'); % covariance of the transformed set {Yi}

%%%% state update done
%%%%% measurement update starts


%%measurement co-variance

%Y_ = Y_sigma';

% for k = 1:size(Y(1:4,:),2)
%    Y_sig(k,:) =   quatinverse(Y(1:4,k)');
% end

gdash = quatmultiply(Y(1:4,:)',quatmultiply(g,quatinv(Y(1:4,:)')));
 g_vector = quaternion_vector(gdash');
 % comparision of the measurement to the observed value from accelerometer
 Z(1:3,:) = g_vector;
 Z(4:6,:) = Y(5:7,:);
 zdash = mean(Z,2);
 Zi = [A(:,i);W(:,i)];
 
 v = Zi - zdash;
 Pzz = cov(Z');
 
 Pvv = Pzz + R;
 
 %%%%cross correlation matrix to compute Pxz
 zz = (bsxfun(@minus,Z,zdash))';
 
 %for j = 1:size(Wdash,2)
 Pxz =  Wdash*zz;
 %end
 Pxz = Pxz/12;
 
 
 %%final updating of the equations
 %%kalman gain update
 
    
  K = Pxz*(Pvv^-1);
 KK = K*v;
  %posterior update
  Dd = quaternion_vector((x_bar(1:4))');
  DDD = Dd + KK(1:3);
  x_hat(1:4) = vector_quaternion(DDD); %%%%checkkk
  %x_hat(1:4)= transpose(quatmultiply((vector_quaternion(KK(1:3)))',(x_bar(1:4))));
  x_hat(5:7) = x_bar(5:7)' +KK(4:6);
 % bla(:,i) = x_hat;
  %P = Pdash - K*Pvv*K';
   P = Pdash - K*Pvv*K';
Qq(:,i) = x_hat(1:4);
%q_fin =  quaternion_vector(Qq(:,i));
% Ar = q_fin;
% yaw =zeros(1,size(Ar,2));
% pitch = atan2(-Ar(1,:),sqrt(Ar(2,:).*Ar(2,:) + Ar(1,:).*Ar(1,:)));
% roll = atan2(-Ar(2,:),Ar(3,:));
% Ra =  angle2dcm(roll,pitch,yaw);
% ukf(:,:,i) = Ra;
ukf(:,:,i) = quat2dcm((Qq(:,i))');
xhist(:,i)=x_hat;

end

  