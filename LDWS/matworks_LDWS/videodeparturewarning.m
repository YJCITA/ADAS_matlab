function [TwoValidLanes, NumNormalDriving, TwoLanes, OutMsg] = ...
            videodeparturewarning(Pts, Imlow, MaxLaneNum, Count_ref, ...
                NumNormalDriving, OutMsg)
% VIDEODEPARTUREWARNING - Detect whether there is a lane departure
% This function is used in the lane departure warning system demo
% videoldws.m.

%   Copyright 2008-2009 The MathWorks, Inc.

% Find the line which intersects with the image bottom boundary to the
% left (right) and which is the closest to the image bottom boundary
% center.
Dis_inf = size(Imlow, 2); % Width of image
Halfwidth = Dis_inf * 0.5;
Left_dis  = single(intmax('int16'));
Left_pts  = zeros(4, 1);
Right_dis = single(intmax('int16'));
Right_pts = zeros(4, 1);
Pts = Pts(:, [2 1 4 3])';

for i = 1:MaxLaneNum
    % Pick the column corresponding to the point closer to the top
    if Pts(1, i) >= Pts(3, i)
        ColNum = Pts(2, i);
    else
        ColNum = Pts(4, i);
    end
    if Count_ref(i) >= 5
        centerDis = abs(Halfwidth - ColNum);
    else
        centerDis = Dis_inf;
    end
    if (Halfwidth - ColNum) >= 0 % left lane
        if centerDis < Left_dis
            Left_dis = centerDis;
            Left_pts = Pts(:, i);
        end
    else                         % right lane
        if centerDis < Right_dis
            Right_dis = centerDis;
            Right_pts = Pts(:, i);
        end
    end
end

% Departure detection
if Left_dis < Dis_inf
    TmpLeftPts = Left_pts;
else
    TmpLeftPts = zeros(4, 1);
end
if Right_dis < Dis_inf
    TmpRightPts = Right_pts;
else
    TmpRightPts = zeros(4, 1);
end
TwoLanes = int32([TmpLeftPts TmpRightPts]);
% Check whether both lanes are valid
Check1 = (TwoLanes(1,:) ~= TwoLanes(3,:)) | ...
         (TwoLanes(2,:) ~= TwoLanes(4,:));
Check2 = (abs(TwoLanes(1,:) - TwoLanes(3,:)) + ...
          abs(TwoLanes(2,:) - TwoLanes(4,:))) >= 10;
TwoValidLanes = (Left_dis <= Dis_inf) && (Right_dis <= Dis_inf) && ...
                 all((TwoLanes(1,:)>=0) & Check1 & Check2);

Diswarn = Dis_inf * 0.4; % Distance threshold for departure warning
if Left_dis < Diswarn && Left_dis <= Right_dis
    RawMsg = 2;
elseif Right_dis < Diswarn && Left_dis > Right_dis
    RawMsg = 0;
else
    RawMsg = 1;
end
% Meaning of Raw Masseage: 0 = Right lane departure,
%                          1 = Normal driving, 2 = Left lane departure

% The following code combines left-right departure to left departure and
% right-left departure to right departure. It utilizes the fact that there
% must be at least 4 frames of normal driving between a left departure
% warning and a right departure warning.
NumNormalDriving = NumNormalDriving + (RawMsg == 1);
RawMsg = int8(RawMsg);
if RawMsg == int8(1) || NumNormalDriving >= 4
    OutMsg = RawMsg;
end % else keep old OutMsg

if RawMsg ~= int8(1)
    NumNormalDriving = 0;
end

TwoLanes = TwoLanes([2 1 4 3], :);
