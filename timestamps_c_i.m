function [ts_i_c, ts_c_i]= timestamps_c_i(ts_c,ts_i)
%cam time and imu time
for  i= 1:size(ts_c,2)
    [ val ,idx] = min(abs(ts_c(i) - ts_i));
    ts_i_c(idx) = i;
    ts_c_i(i) = idx;
end