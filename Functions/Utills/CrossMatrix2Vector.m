function[Vec]=CrossMatrix2Vector(A)

if size(A,1)~=3
    error('A Must Have 3 Rows')
elseif size(A,2)~=3
    error('A Must Have 3 Coloumns')
end
Vec=zeros(1,3);
Vec(1,1)=A(3,2);
Vec(1,2)=A(1,3);
Vec(1,3)=A(2,1);
end