function [xhatbar eV] = gradientDescent(q_next,q_prev)
	q_prev = q_prev';
	count = 0;
	cost = ones(1,1);
	thresh = 0.0001;
	while(cost>thresh && count < 100)
		%q12 = q2*inv(q1)%Calculated in quaternions
		eQ = quatmultiply(q_next,quatinv(q_prev));
		%calculate mean
		%convert to vector form first and den calculate mean
		eV = getvect(eQ');
		Mean = mean(eV,2);
		%Convert back to quat and multiply with q_prev to calculate new q
		q_prev = quatmultiply((getquat(Mean))',q_prev);
		%Update Cost whih has to be minimized
		cost = sqrt(sum(Mean.*Mean));
		%Update iterations
		count = count+1;
	end
	xhatbar = q_prev';
	%display('Ran Gradient Descent')

