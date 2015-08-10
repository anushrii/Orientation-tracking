function q=vector_quaternion(input_vector)


    norm=sqrt(sum(input_vector(1:3,:).*input_vector(1:3,:),1));
    alpha=norm;
    
    e=bsxfun(@rdivide,input_vector(1:3,:),norm);
    e(isnan(e))=0;
    e_vec = bsxfun(@times,e,sin(alpha/2));
    q=[cos(alpha/2); e_vec ];
end