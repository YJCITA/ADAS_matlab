function colorAndTypeIdx = videodetectcolorandtype(twoLines_0b, YCbCr)  
% VIDEODETECTCOLORANDTYPE - Detects if the lane detected is Yellow/White
% and Solid/Broken in videoldws demo.

%   Copyright 2008-2009 The MathWorks, Inc.

% twoLines_0b =  coordinate points of lines (0 based) 
%
% Here, for a line with start point (r1,c1) and end point (r2,c2),
% we form a ROI with points: 
% (r1,c1-HALF_OFFSET) (r1,c1+HALF_OFFSET) 
% (r2,c2-HALF_OFFSET) (r2,c2+HALF_OFFSET) 
% We search for yellow/white color within this ROI 

INVALID_COLOR = int8(0);
WHITE_COLOR   = int8(1);
YELLOW_COLOR  = int8(2); 

INVALID_MARKING = int8(0);
BROKEN_MARKING  = int8(1); 
SOLID_MARKING   = int8(2);

INVALID_COLOR_OR_TYPE = int8(1);% color=yellow/white; type=broken or solid
YELLOW_BROKEN  = int8(2); 
YELLOW_SOLID   = int8(3); 
WHITE_BROKEN   = int8(4); 
WHITE_SOLID    = int8(5); 

lineColorIdx    = int8([0 0]);
solidBrokenIdx  = int8([0 0]);
colorAndTypeIdx = int8([0 0]);
HALF_OFFSET =  int32(10);

zeroI32 = int32(0);
oneI32  = int32(1);
twoI32  = int32(2);

rH = int32(size(YCbCr(:,:,1),1));
cW = int32(size(YCbCr(:,:,1),2));

tmpInImage_Y  = YCbCr(:,:,1);
tmpInImage_Cb = YCbCr(:,:,2);

leftC  = int32(zeros(rH,1));
rightC = int32(zeros(rH,1));
Rs     = int32(zeros(rH,1));

twoLines_1b = twoLines_0b([2 1 4 3], :);
for i=oneI32:int32(length(twoLines_1b(:)))
    twoLines_1b(i) = twoLines_1b(i) + oneI32;
end

for lineIdx =oneI32:twoI32%1:2
    %% one line
    r1c1r2c2 = twoLines_1b(:,lineIdx) - 1;
    r1 = r1c1r2c2(1); 
    c1 = r1c1r2c2(2); 
    r2 = r1c1r2c2(3); 
    c2 = r1c1r2c2(4); 

    % make sure r1 is the min (r1,r2)
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
    if ((r1>zeroI32 && r1 <= rH) && (c1>zeroI32 && c1 <= cW) && ...
        (r2>zeroI32 && r2 <= rH) && (c2>zeroI32 && c2 <= cW)) && ~pointNotLine
        line_within_image = logical(true);
    else
        line_within_image = logical(false);
    end

    if (line_within_image)
        len = r2-r1+oneI32;
        i=oneI32;
        % Rs(1:len)=r1:r2
        for p=r1:r2
          Rs(i) = p;
          i=i+oneI32;
        end
        %leftC(1:len) = int32(linspace(c1-HALF_OFFSET, c2-HALF_OFFSET, len));
        %y = linspace(d1, d2, n)
        %y = [d1+(0:n-2)*(d2-d1)/(floor(n)-1) d2];
        
        quotient = (single(c2-c1))/(single(len)-single(1) + single(eps));%(d2-d1)/(floor(n)-1)
        for i=oneI32:len
            leftC(i)  = (c1-HALF_OFFSET) + int32((single(i)-single(1))*quotient);
            rightC(i) = leftC(i)+ twoI32*HALF_OFFSET;
            if leftC(i) < oneI32
                leftC(i) = oneI32;
                if rightC(i) < oneI32
                   rightC(i) = oneI32;
                end
            end
            if rightC(i) > cW
                rightC(i) = cW;
            end
        end
            
        whiteCount  = zeroI32;
        yellowCount = zeroI32;
        grayCount   = zeroI32;

        SumOfGotAlLeastOneWhitePixelInTheRow  = zeroI32;
        SumOfGotAlLeastOneYellowPixelInTheRow = zeroI32;
            
        for i=oneI32:len
            gotAlLeastOneWhitePixelInTheRow  = logical(false);
            gotAlLeastOneYellowPixelInTheRow = logical(false);
            for cv = leftC(i):rightC(i)
              
                   if tmpInImage_Y(Rs(i), cv) >= single(175/255)
                       whiteCount = whiteCount+oneI32;
                       gotAlLeastOneWhitePixelInTheRow = logical(true);
                   elseif tmpInImage_Cb(Rs(i), cv) >= single(90/255) && tmpInImage_Cb(Rs(i), cv) <= single(127/255) 
                       yellowCount = yellowCount+oneI32;
                       gotAlLeastOneYellowPixelInTheRow = logical(true);
                   else
                       grayCount = grayCount+oneI32;
                   end
            end
            if gotAlLeastOneWhitePixelInTheRow
              SumOfGotAlLeastOneWhitePixelInTheRow = SumOfGotAlLeastOneWhitePixelInTheRow+oneI32;
            end
            if gotAlLeastOneYellowPixelInTheRow
              SumOfGotAlLeastOneYellowPixelInTheRow = SumOfGotAlLeastOneYellowPixelInTheRow+oneI32;
            end
        end

        yellowVsTotal = single(yellowCount)/single(grayCount+yellowCount+whiteCount);
        whiteVsTotal  = single(whiteCount)/single(grayCount+yellowCount+whiteCount);
         
        if yellowVsTotal > whiteVsTotal
            lineColorIdx(lineIdx) = YELLOW_COLOR;
            linearPixelRatio = single(SumOfGotAlLeastOneYellowPixelInTheRow)/single(len);             
        else
            lineColorIdx(lineIdx) = WHITE_COLOR;
            linearPixelRatio = single(SumOfGotAlLeastOneWhitePixelInTheRow)/single(len);
        end
        
        %% distinguish solid and broken lane markers
        % if we use yellowVsTotal(==whiteVsTotal) to find marker type (solid/broken)
        % it would not give accurate result; because in the patch we are searching 
        % for yellow/white pixel- we do not know what's the pecentage of gray pixels 
        % are there
        % that's why we are considering a different method; here we try to find one
        % yellow (==white) pixel in one row; once we find a yellow (==white) pixel
        % in the row we don't need to find anything in that row of the patch (ROI)

        if linearPixelRatio > single(0.8)
             solidBrokenIdx(lineIdx) = SOLID_MARKING;
        else
             solidBrokenIdx(lineIdx) = BROKEN_MARKING;
        end 
        
        if (lineColorIdx(lineIdx) == YELLOW_COLOR) && (solidBrokenIdx(lineIdx) == BROKEN_MARKING)
            colorAndTypeIdx(lineIdx) = YELLOW_BROKEN;
        elseif (lineColorIdx(lineIdx) == YELLOW_COLOR) && (solidBrokenIdx(lineIdx) == SOLID_MARKING)
            colorAndTypeIdx(lineIdx) = YELLOW_SOLID;
        elseif (lineColorIdx(lineIdx) == WHITE_COLOR) && (solidBrokenIdx(lineIdx) == BROKEN_MARKING)
            colorAndTypeIdx(lineIdx) = WHITE_BROKEN;
        elseif (lineColorIdx(lineIdx) == WHITE_COLOR) && (solidBrokenIdx(lineIdx) == SOLID_MARKING)
            colorAndTypeIdx(lineIdx) = WHITE_SOLID;
        end
        
    else
        lineColorIdx(lineIdx) = INVALID_COLOR;
        solidBrokenIdx(lineIdx) = INVALID_MARKING;
        colorAndTypeIdx(lineIdx) = INVALID_COLOR_OR_TYPE;
    end
end

