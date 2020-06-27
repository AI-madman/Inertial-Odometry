function [G] = CaculateGMatrix(imuState_k)
%Caculates the Matrix that Propegates the Noise
% The function takes as input a struct of the current imu state 
% and extracts the rotation of the imu state, then procceds to convert the 
% rotation to a Rotation Matrix from a Quertion. Then it caculates the G
% Matrix as in A Multi-State Constraint Kalman Filter
%for Vision-aided Inertial Navigation.
G = zeros(12,12);
    
C_IG = Quaterion2RotationMatrix(imuState_k.q_IG);
    
G(1:3,1:3) = -eye(3);
G(4:6,4:6) = eye(3);
G(7:9,10:12) = eye(3);
G(10:12,7:9) = -C_IG';
end

