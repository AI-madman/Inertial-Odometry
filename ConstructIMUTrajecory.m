IMUPostion=zeros(3,1057);
for i=1:1:1057
    IMUPostion(:,i)=KFState_IMUOnly{i}.imuState.p_I_G;
end
IMUPlot=figure();
scatter3(IMUPostion(1,:),IMUPostion(2,:),IMUPostion(3,:));
