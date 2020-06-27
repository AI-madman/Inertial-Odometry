function [F] = CaculateFMatrix(imuState_k,measurements_k)
%CACULATEFMATRIX Returns the State Propegation Matrix
%   The F Matrix has a standered form See MSCKF Paper, however,
%   the matrix requires the input of the measurements of the IMU 
%   state at the kth time step to caculate the K+1th state.
%   This is in continous time.

    F = zeros(12,12);
    
    omegaHat = measurements_k.omega - imuState_k.b_g;
    vHat = measurements_k.v - imuState_k.b_v;
    C_IG = Quaterion2RotationMatrix(imuState_k.q_IG);
    
    F(1:3,1:3) = -Vec2CrossMatrix(omegaHat);
    F(1:3,4:6) = -eye(3);
    F(10:12,1:3) = -C_IG' * Vec2CrossMatrix(vHat);
    F(10:12,7:9) = -C_IG';
end

