function Q = quatinverse(q)

q = reshape( q, 4, 1 );
conjq = [q(1);-q(2:4)];
Q = conjq ./ (sqrt( sum( q .* q )))^2;
