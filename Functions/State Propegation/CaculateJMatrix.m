function J = CaculateJMatrix(camera, imuState_k)

    C_CI = Quaterion2RotationMatrix(camera.q_CI);
    C2_CI = Quaterion2RotationMatrix(camera.q2_CI);
    C_IG = Quaterion2RotationMatrix(imuState_k.q_IG);



J = zeros(12, 12);
J(1:3,1:3) = C_CI;
J(4:6,1:3) = Vec2CrossMatrix(C_IG' * camera.p_C_I);
J(4:6,10:12) = eye(3);
J(7:9,1:3) =  C2_CI;
J(10:12,1:3) = Vec2CrossMatrix(C_IG' * camera.p2_C_I );
J(10:12,10:12) = eye(3);
end
