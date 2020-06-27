function oxts = LoadOxts(base_dir,TimeStamps)
% reads GPS/IMU data from files to memory. requires base directory
% (=sequence directory as parameter). if frames is not specified, loads all frames.

ts=TimeStamps;

  oxts  = [];
  for i=1:length(ts)
    if ~isempty(ts{i})
      oxts{i} = dlmread([base_dir '/oxts/data/' num2str(i-1,'%010d') '.txt']);
    else
      oxts{i} = [];
    end
  end
   

end


