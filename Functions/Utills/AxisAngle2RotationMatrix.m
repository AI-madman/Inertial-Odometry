function[B]=AxisAngle2RotationMatrix(A)
    np = norm(A);
    cp = cos(np);
    sp = sin(np);
    pnp = A / np;
    B = cp * eye(3) + (1 - cp) * (pnp * pnp') - sp * Vec2CrossMatrix((pnp));
    if isnan(B)
       B=zeros(3,3); 
    end

end