function op_vector=quaternion_vector(input_quaternion)


    op_vector=bsxfun(@rdivide,input_quaternion(2:4,:),sqrt(1-input_quaternion(1,:).*input_quaternion(1,:)));
    op_vector(isnan(op_vector))=0;
    
    
end