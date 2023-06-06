function [cxy, c45, Ix, Iy, Ixy, I_45_45] = secondDerivCornerMetric(I, sigma, highDistortion)
%#codegen

%   Copyright 2013-2020 The MathWorks, Inc.

% Low-pass filter the image
G = fspecial('gaussian', round(sigma * 7)+1, sigma);
Ig = imfilter(I, G, 'conv');

derivFilter = [-1 0 1];

% first derivatives
Iy = imfilter(Ig, derivFilter', 'conv');
Ix = imfilter(Ig, derivFilter, 'conv');

% second derivative
Ixy = imfilter(Ix, derivFilter', 'conv');

c45 = zeros(size(I), 'like', I);
I_45_45 = zeros(size(I), 'like', I);

if ~highDistortion
    % define steerable filter constants
    cosPi4 = coder.const(cast(cos(pi/4), 'like', I));
    cosNegPi4 = coder.const(cast(cos(-pi/4), 'like', I));
    sinPi4 = coder.const(cast(sin(pi/4), 'like', I));
    sinNegPi4 = coder.const(cast(sin(-pi/4), 'like', I));

    % first derivative at 45 degrees
    I_45 = Ix * cosPi4 + Iy * sinPi4;
    I_n45 = Ix * cosNegPi4 + Iy * sinNegPi4;


    I_45_x = imfilter(I_45, derivFilter, 'conv');
    I_45_y = imfilter(I_45, derivFilter', 'conv');

    I_45_45 = I_45_x * cosNegPi4 + I_45_y * sinNegPi4;

    % suppress the outer corners
    cxy = sigma^2 * abs(Ixy) - 1.5 * sigma * (abs(I_45) + abs(I_n45));
    cxy(cxy < 0) = 0;
    c45 = sigma^2 * abs(I_45_45) - 1.5 * sigma * (abs(Ix) + abs(Iy));
    c45(c45 < 0) = 0;

else % Use additional filters for images with high distortion
    % Filter kernel design
    % Example -> For range: [45 -45], filterSize = 5, the 4 kernels are:
    % [ 0  0  0  0  0   [0  1  1  1  0   [0  0  0  0  0   [ 0 -1 -1 -1  0
    %  -1  0  0  0  1    0  0  1  0  0    1  0  0  0 -1     0  0 -1  0  0
    %  -1 -1  0  1  1    0  0  0  0  0    1  1  0 -1 -1     0  0  0  0  0
    %  -1  0  0  0  1    0  0 -1  0  0    1  0  0  0 -1     0  0  1  0  0
    %   0  0  0  0  0]   0 -1 -1 -1  0]   0  0  0  0  0]    0  1  1  1  0]
    filterAngRanges = [ 0   45  0;
                      -90 -45 -45];

    filterSizes = 5;
    derivFilters = getDerivFilters(filterAngRanges, filterSizes);

    checkerboardPeaks = zeros(size(I), 'like', I);
    numFilters = size(derivFilters, 1);

    IfilterDot = zeros(size(Ig,1), size(Ig,2), 4, 'like', I);
    IfilterDdot = zeros(size(Ig,1), size(Ig,2), 4, 'like', I);

    for filtIdx = 1:numFilters

        % First derivatives
        IfilterDot(:,:,1) = imfilter(Ig, derivFilters{filtIdx, 1}, 'conv');
        IfilterDot(:,:,2) = imfilter(Ig, derivFilters{filtIdx, 2}, 'conv');
        IfilterDot(:,:,3) = imfilter(Ig, derivFilters{filtIdx, 3}, 'conv');
        IfilterDot(:,:,4) = imfilter(Ig, derivFilters{filtIdx, 4}, 'conv');

        % Second derivatives
        IfilterDdot(:,:,1) = imfilter(IfilterDot(:,:,1), derivFilters{filtIdx, 1}, 'conv');
        IfilterDdot(:,:,2) = imfilter(IfilterDot(:,:,2), derivFilters{filtIdx, 2}, 'conv');
        IfilterDdot(:,:,3) = imfilter(IfilterDot(:,:,1), derivFilters{filtIdx, 3}, 'conv');
        IfilterDdot(:,:,4) = imfilter(IfilterDot(:,:,2), derivFilters{filtIdx, 4}, 'conv');

        checkerEdge = IfilterDdot(:,:,1) + IfilterDdot(:,:,2) - (abs(Ix).*0.3 + abs(Iy).*0.3);
        checkerEdgeComp = IfilterDdot(:,:,3) + IfilterDdot(:,:,4) - (abs(Ix).*0.3 + abs(Iy).*0.3);

        checkerEdge(checkerEdge<0) = 0;
        checkerEdgeComp(checkerEdgeComp<0) = 0;

        % Dilate to prevent loss of significant points
        se = strel('disk',1);
        checkerEdge = imdilate(checkerEdge,se);
        checkerEdgeComp = imdilate(checkerEdgeComp,se);

        % Implicit expansion not supported in simulation
        if isempty(coder.target)
            checkerPts12 = max(IfilterDdot(:,:,1:2) - checkerEdge, 0);
            checkerPts34 = max(IfilterDdot(:,:,3:4) - checkerEdgeComp, 0);
        else
            checkerPts12 = max(bsxfun(@minus, IfilterDdot(:,:,1:2), checkerEdge), 0);
            checkerPts34 = max(bsxfun(@minus, IfilterDdot(:,:,3:4), checkerEdgeComp), 0);
        end
        
        checkerboardPeaks = checkerboardPeaks + sum(checkerPts12,3) + sum(checkerPts34,3);

    end

    % Erosion to get better corner localization accuracy
    se = strel('disk',1);
    cxy = imerode(checkerboardPeaks, se);

end

%--------------------------------------------------------------------------
% getDerivFilters - Get filter kernels for specified angles and sizes
%--------------------------------------------------------------------------
function filters = getDerivFilters(angRange, filtSizes)

diffAngles = angRange(1,:) - angRange(2,:);
angRange = mod(angRange + 360, 360);

numRanges = length(angRange);
numSizes = length(filtSizes);

filters = cell(numRanges * numSizes, 4);

for sizeIdx = 1:numSizes % Filter kernel sizes

    filtSize = filtSizes(sizeIdx);

    start = (sizeIdx-1)*numRanges + 1;
    stop  = sizeIdx*numRanges;
    numPrototypes = 4;
    for idxFilterAngle = start:stop
        for idxPrototype = 1:numPrototypes
            filters{idxFilterAngle,idxPrototype} = zeros(filtSize,filtSize);
        end
    end

    [idX, idY] = meshgrid(1:filtSize, 1:filtSize);

    filterMidPt = ceil(filtSize/2);
    idTheta = atan2(idY - filterMidPt, idX - filterMidPt);

    idTheta = rad2deg(idTheta);
    idTheta = mod(idTheta + 360, 360);

    for rangeIdx = 1:numRanges % Angle ranges

        idx = (sizeIdx - 1)*numRanges + rangeIdx;
        range = angRange(:,rangeIdx);

        rangeComp = (180 - diffAngles(rangeIdx))/2;
        rangeComp = range + [2*rangeComp; -2*rangeComp];

        thetaCand = [range; rangeComp];

        % Filter prototype 1
        filters{idx, 1}(idTheta < thetaCand(1) | idTheta > thetaCand(2)) = 1;
        filters{idx, 1}(idTheta > thetaCand(3) & idTheta < thetaCand(4)) = -1;

        filters{idx, 1}(filterMidPt, filterMidPt) = 0;

        % Filter prototype 2
        filters{idx, 2}(idTheta > thetaCand(4) & idTheta < thetaCand(2)) = 1;
        filters{idx, 2}(idTheta > thetaCand(1) & idTheta < thetaCand(3)) = -1;

        filters{idx, 2}(filterMidPt, filterMidPt) = 0;

        % Filter prototype 3
        filters{idx, 3} = -filters{idx, 1};

        % Filter prototype 4
        filters{idx, 4} = -filters{idx, 2};

    end

end
