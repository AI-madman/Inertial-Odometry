function[IMUState2D,P,KalmanGain]=ComputeOptimalFusion(KFState_IMUOnly,VO,P)
% Computes the EKF fusion of IMU and VO pose estimations.
% Based of the state vector degined in Axcels PhD thesis.
% H is the extraction matrix derived from the PhD state vector.
H=zeros(6,15);
H=zeros(24);
H(1:3,1:3)=eye(3);
H(4:6,7:9)=eye(3);
R=zeros(24,24);
IMUState2D=[KFState_IMUOnly.CamStates.p_C_G' quat2angle(KFState_IMUOnly.CamStates.q_CG')]';
innovation=IMUState2D-[VO.p_C1_G' quat2angle(VO.q_C1_G')]';
innovationCovariance= H*P*H' +R;
KalmanGain=P*H'*inv(innovationCovariance);
%IMUState2D=IMUState2D+KalmanGain*innovation;
%P=(eye(12)-KalmanGain*H)*P*(eye(12)-KalmanGain*H)'+ KalmanGain*R*KalmanGain;
Ky=KalmanGain*innovation;
Ky=Ky(1:4,1);
IMUState2D=IMUState2D+Ky;
P=(eye(24)-KalmanGain*H)*P*(eye(24)-KalmanGain*H)';
end