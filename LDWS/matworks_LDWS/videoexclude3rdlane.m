function out3rdLane = videoexclude3rdlane(in3rdLane, Show3rdLane, mainTwoLanes, twoLanesFlag, RefImg)  
% VIDEOEXCLUDE3RDLANE: Exclude third lane in videoldws demo

%   Copyright 2008-2009 The MathWorks, Inc.

% Exclude third (left or right lane) if it is very close to the other lane 
% already detected and tracked by Kalman filter.

[rH cW] = size(RefImg(:,:,1));
out3rdLane = in3rdLane;  

if (Show3rdLane && twoLanesFlag)
    R1C1R2C2 = in3rdLane;
    R1 = R1C1R2C2(1)+int32(1); 
    C1 = R1C1R2C2(2)+int32(1); 
    R2 = R1C1R2C2(3)+int32(1);  
    C2 = R1C1R2C2(4)+int32(1);  
    % make sure R1 is the min(R1,R2)
    if R1>R2
        TMP=R2;
        R2=R1;
        R1=TMP;

        TMP=C2;
        C2=C1;
        C1=TMP;   
    end
    
  for lineIdx =1:2  
    
    r1c1r2c2 = mainTwoLanes(:,lineIdx);
    r1 = r1c1r2c2(1)+int32(1);  
    c1 = r1c1r2c2(2)+int32(1);  
    r2 = r1c1r2c2(3)+int32(1);  
    c2 = r1c1r2c2(4)+int32(1); 

    % make sure r1 is the min(r1,r2)
    if r1>r2
        tmp=r2;
        r2=r1;
        r1=tmp;

        tmp=c2;
        c2=c1;
        c1=tmp;   
    end
    
    pointNotLine = (r1==r2) && (c1==c2);
    
    % find if line is within image: (r1,c1)  and (r2,c2) must be within image
    if ((r1>0 && r1 <= rH) && (c1>0 && c1 <= cW) && ...
        (r2>0 && r2 <= rH) && (c2>0 && c2 <= cW)) && ~pointNotLine
        %line_within_image = true;
        if (abs(r1-R1)+ abs(c1-C1) + abs(r2-R2) +abs(c2-C2))< int32(20)
            out3rdLane = int32([-1000 -1000 -1000 -1000]');
        end            
    else
       % line outside image
       % go to nextLine
    end    
  end    
else
   out3rdLane = in3rdLane;  
end



