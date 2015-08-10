function [ts_c_v, ts_v_c]= timestamps_v_c(ts_v,ts_c)
%cam time and vicon time
for  i= 1:size(ts_c,2)
    [ val ,idx] = min(abs(ts_c(i) - ts_v));
    ts_v_c(idx) = i;
    ts_c_v(i)=idx;
end
