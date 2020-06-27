function[Omega]=CaculateOmegaMatrix(w)

Omega=zeros(4,4);
Omega(1:3,1:3)=-Vec2CrossMatrix(w.');
Omega(4,1:3)=-w;
Omega(1:3,4)=transpose(w);

end