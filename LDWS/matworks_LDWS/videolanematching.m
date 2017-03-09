function [Rep_ref, Count_ref] = videolanematching(Rep_ref, Count_ref, ...
                                   MaxLaneNum, ExpLaneNum, Enable, Line, ...
                                   TrackThreshold, CountUpperThresh)
% VIDEOLANEMATCHING - Tracks the lane marker lines
% This function is used in the lane departure warning system demo
% videoldws.m.

%   Copyright 2008-2009 The MathWorks, Inc.

% Lane matching
% Calculate the distances between the lines found in the current frame
% and those in the repository.
List = double(intmax('int16')) * ones(MaxLaneNum, ExpLaneNum);
for i = 1:MaxLaneNum
    for j = 1:ExpLaneNum
        if Count_ref(i) > 0 && Enable(j) == 1
            List(i, j) = abs(Line(1, j)' - Rep_ref(1,i)) + ...
                abs(Line(2, j)' - Rep_ref(2, i)) * 200;
        end
    end
end
% Find the best matches between the current lines and those in the
% repository.
%Match_dis  = intmax('int16')*ones(1, MaxLaneNum, 'int16');
Match_dis  = double(intmax('int16'))*ones(1, MaxLaneNum);
Match_list = zeros(2, MaxLaneNum);
for i = 1:ExpLaneNum
    if i > 1
        % Reset the row and column where the minimum element lies on.
        List(rowInd, :) = double(intmax('int16')) * ones(1, ExpLaneNum);
        List(:, colInd) = double(intmax('int16')) * ones(MaxLaneNum, 1);
    end
    % In the 1st iteration, find minimum element (corresponds to
    % best matching targets) in the distance matrix. Then, use the
    % updated distance matrix where the minimun elements and their
    % corresponding rows and columns have been reset.
    [Val, Ind]      = min(List(:));
    [rowInd, colInd] = ind2sub(size(List), Ind);
    Match_dis(i)    = Val;
    Match_list(:,i) = [rowInd colInd]';
end
% Update reference target list.
% If a line in the repository matches with an input line, replace
% it with the input one and increase the count number by one;
% otherwise, reduce the count number by one. The count number is
% then saturated.
Count_ref = Count_ref - 1;
for i = 1:ExpLaneNum
    if Match_dis(i) > TrackThreshold
        % Insert in an unused space in the reference target list
        NewInd = find(Count_ref < 0, 1);
        Rep_ref(:, NewInd) = Line(:, Match_list(2, i));
        Count_ref(NewInd) = Count_ref(NewInd) + 2;
    else
        % Update the reference list
        Rep_ref(:, Match_list(1, i)) = Line(:, Match_list(2, i)); 
        Count_ref(Match_list(1, i)) = Count_ref(Match_list(1, i)) + 2;
    end
end
Count_ref(Count_ref < 0) = 0;
Count_ref(Count_ref > CountUpperThresh) = CountUpperThresh;
