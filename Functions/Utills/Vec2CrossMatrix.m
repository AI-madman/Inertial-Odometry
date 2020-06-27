function [C] = Vec2CrossMatrix(A)
%VEC2CROSSMATRIX Computes the cross matrix of A
%   This function takes as input a single 1x3 vector and 
%   determines the cross Matrix corrponding to the vector.
if size(A) == [3,1]
   A=A';
   disp('The input matrix has been transposed to work with the functions dimentions')
end
if size(A,1)~=1
    error('A Must Have One Row')
end
if size(A,2)~=3
    error('A Must Have 3 Coloumns')
end
if ndims(A)~=2
    error('A Must be a 2 Dimentional array')
end
C=zeros(3,3);
C(1,2)=-A(1,3);
C(1,3)=A(1,2);
C(2,1)=A(1,3);
C(2,3)=-A(1,1);
C(3,1)=-A(1,2);
C(3,2)=A(1,1);

end

