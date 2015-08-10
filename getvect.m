function vect = getvect(data)
	vect = bsxfun(@rdivide,data(2:4,:),sqrt(1-data(1,:).*data(1,:)));
	vect(isnan(vect)) = 0;

