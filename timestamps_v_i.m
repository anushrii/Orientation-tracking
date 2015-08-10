function [ts_i_v, ts_v_i]= timestamps_v_i(ts_v,ts_i)
for  i= 1:size(ts_v,2)
    [ val ,idx] = min(abs(ts_v(i) - ts_i));
    ts_i_v(idx) = i;
    ts_v_i(i) = idx;
end
