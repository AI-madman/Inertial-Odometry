function [timestamps] = LoadImageTimeStamps(framesFolder, imageRange)
% Returns TimeStamp of each image in sequance
% Pass in the folder with the images and the number of images and the 
% function will read the first n=imageRange of images and find the time 
% at which each image was taken and pass the results back.

    % Read all image timestamps
    dateStrings = loadTimestamps(framesFolder);
    dateStrings = dateStrings(imageRange);
    timestamps = zeros(1, length(dateStrings));
    for i = 1:length(dateStrings)
        timestamps(i) =  datenum_to_unixtime(datenum(dateStrings(i)));
    end
 
    
    function dn = datenum_to_unixtime( date_num )
      dn =  (date_num - 719529)*86400;         %# 719529 == datenum(1970,1,1)
    end
end

