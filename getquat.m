function quat = getquat(data)
	alpha = sqrt(sum(data.*data));
	Axis = bsxfun(@rdivide,data,alpha);
	Axis(isnan(alpha)) = 0;
	quat = [cos(alpha/2); bsxfun(@times,Axis,sin(alpha/2))];
	quat(isnan(quat)) = 0;

	
