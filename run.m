%%%%vicon stuff
clear all
close all
clc

% adding path to the test folder and it's subfolders
addpath(genpath('C:\Users\sabhajit singh\Desktop\learning in robotics\project2\test'))

% change the file number to be loaded here.
n = 10; % file number that needs to be loaded
vicon = ['viconRot' num2str(n) '.mat'];
imu = ['imuRaw' num2str(n) '.mat'];
cam = ['cam' num2str(n) '.mat'];


%%%%comment the VICON load if not present!!!

load (vicon)
ts_v = ts;


% uncomment for making mosaic
% load (cam)
% ts_c = ts;


load(imu)
ts_i = ts;
a = vals;

%%%the time stamps of these sets are not in sync
[A , W, Ra, Rq, Euler_A , Euler_W] =  setright_IMU(vals);

%%%%denoting data as according to timestamps
%vicon time and imu time

%%%time stamps matching

% uncomment the third and fourth for making mosaics.
%comment the first one if there are no VICON data present
[ts_i_v, ts_v_i]= timestamps_v_i(ts_v,ts_i);
% [ts_i_c, ts_c_i]= timestamps_c_i(ts_c,ts_i);
% [ts_c_v, ts_v_c]= timestamps_v_c(ts_v,ts_c);


[Qq, ukf] = get_ukf(A, W, ts_i);


% for seven state
% [Qq, ukf] = get_ukf_7(A, W);



%uncomment for making video!!!!!!!
% fig=figure;
% aviobj=avifile('video of UKF & Vicon comparision data set 8','fps',10);
% for i=1000:5:3000
%     if (abs(ts_i(ts_v_i(i))-ts_v(i))<0.01)
%         rot_plot(ukf(:,:,ts_v_i(i)),rots(:,:,i),i);
%         g=getframe(fig);
%         aviobj=addframe(aviobj,g);
%     end
% end
% aviobj=close(aviobj);


% fig=figure;
% aviobj=avifile('video of UKF & Vicon comparision test set 10','fps',10);
for i=1000:5:3000
    if (abs(ts_i(ts_v_i(i))-ts_v(i))<0.01)
        
      % to plot only the UKF uncomment this!!!!!!
      % rot_plot_ukf(ukf(:,:,i),i);
      
      % to plot a comparison of UKF and VICON
       rot_plot(ukf(:,:,ts_v_i(i)),rots(:,:,i),i);
       
       
       %g=getframe(fig);
      %aviobj=addframe(aviobj,g);
   end
end
 %aviobj=close(aviobj);


% for i=1000:3000
%     if (abs(ts_i(ts_v_i(i))-ts_v(i))<0.01)
% rot_plot_comparision(ukf(:,:,ts_v_i(i)),Ra(:,:,ts_v_i(i)), Rq(:,:,ts_v_i(i)),i);
%     end
% end
  
%  mosaic()
%  function mosaic( ts_c, ts_i, ts_c_i, ukf, cam)
% 
% mosaic=zeros(2500,2500,3);
% figure,
% for i=1:numel(ts_c)
%     if (abs(ts_c(i)-ts_i(ts_c_i(i)))<0.1)
%        Image=cam(:,:,:,i);  
%         R = ukf(:,:,ts_c_i(i));
%         y=asin(R(1,3));
%         z=acos(R(1,1)/cos(y));
%         x=acos(R(3,3)/cos(y));
%           
%    
%     Image=double(imrotate(Image,x*180/pi));
%     
%     right = 200*tan(-y) - floor(size(Image,1)/2) + 750;
%     position = 200*z - floor(size(Image,2)/2) + 750;
%     r1 = right+size(Image,1)-1;
%     p1 = position+size(Image,2)-1;
%     index=(Image>0);
%     if (right>=1 && right<=size(mosaic,1)-size(Image,1)+1 && position>=1 && position<=size(mosaic,2)-size(Image,2)+1)
%         mosaic(right:r1,position:p1,:)=uint8(((1-index).*mosaic(right:r1,position:p1,:)+index.*Image(:,:,:)));
%        imshow(uint8(mosaic));
%         
%     end
%     end
% end
% 
% 







