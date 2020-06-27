function[C] = Quaterion2RotationMatrix(Q)

 if( size(Q,1) ~= 4 || size(Q,2) ~= 1 )
        error('Input quaternion must be 4x1');
 end
    
    if( abs(norm(Q) - 1) > eps )
        if abs(norm(Q) - 1) > 0.1
            warning(sprintf('Input quaternion is not unit-length. norm(q) = %f. Re-normalizing.', norm(Q)));
        end
        Q = Q/norm(Q);
    end
    
    R = quatRightComp(Q)' * quatLeftComp(Q);
    RN=R(1:3,1:3);
    C = NormaliseMatrix(RN);
      
end






