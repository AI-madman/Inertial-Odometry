TotalDistanceTraveled=0;
for i=2:1:length(GPSData.X)
    TotalDistanceTraveled=TotalDistanceTraveled+sqrt(square(GPSData.X(i)-GPSData.X(i-1))+square(GPSData.Y(i)-GPSData.Y(i-1))+square(GPSData.Z(i)-GPSData.Z(i-1)));
end