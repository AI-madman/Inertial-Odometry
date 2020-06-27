function[NM]=NormaliseMatrix(A)
disp(A)
[U,S,V]=svd(A);
NM=U*eye(size(S,1))*transpose(V);
end