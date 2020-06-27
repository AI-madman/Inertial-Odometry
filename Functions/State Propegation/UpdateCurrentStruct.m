function[KFState_IMUOnly]=UpdateCurrentStruct(KFState_IMUOnly,IMUState2D,P)
% Updates the Current State with the filter values

      KFState_IMUOnly.CamStates.p_C_G=IMUState2D(1:3,1);
      KFState_IMUOnly.CamStates.q_C_G=angle2quat(IMUState2D(4,1),IMUState2D(5,1),IMUState2D(6,1));
      KFState_IMUOnly.imuState.q_IG=RotationMatrix2Quaterion(RigidTransform.Cam_IMU.Rotation*(Quaterion2RotationMatrix(KFState_IMUOnly.CamStates.q_C_G)));
      KFState_IMUOnly.imuState.p_I_G=Quaterion2RotationMatrix(KFState_IMUOnly.imuState.q_IG)+KFState_IMUOnly.CamStates.p_C_G;

      KFState_IMUOnly.imuCovar=P(1:12,1:12);
      KFState_IMUOnly.imuCamCovar=P(1:12,13:24);
      KFState_IMUOnly.KFState.camCovar=P(13:24,13:24);
end