%{
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io> and Pranav Bhounsule
%}
function s = open_log(fname)

if ~exist(fname,'file')
  error('File not found: %s', fname)
end

fileID = fopen(fname);
colnames = fgetl(fileID);
fmt = fgetl(fileID);
fprintf('Opened %s\nKeys: %s\nFormat: %s\n', fname, colnames, fmt)

colnames = strsplit(colnames, ',');
data = zeros([0,length(colnames)]);
i = 0;

try
  while ~feof(fileID)
    alignment1 = fread(fileID,1,'uint8');
    if alignment1 ~= 170
      continue;
    end
    alignment2 = fread(fileID,1,'uint8');
    if alignment2 ~= 187
      continue;
    end

    i = i+1;
     
    for j=1:length(fmt)
      if (fmt(j)=='f')
        type = 'float';
      elseif (fmt(j)=='I')
        type = 'uint32';
      elseif (fmt(j) =='B')
        type = 'uint8';
      else
        error('unspecfied datatype in fmt');
      end

      data(i,j)=fread(fileID,1,type);
    end
  end
  fclose(fileID);
catch
  disp('Finished reading');
end

% Return data in a struct
s = struct();
for cn=1:length(colnames)
  % convert time to seconds
  if strcmp(colnames{cn},'t')
    data(:,cn) = 0.001 * data(:,cn);
  end
  s = setfield(s,colnames{cn},data(:,cn));
end

end
