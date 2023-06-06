function points = circularBoundary(imagePoints, I)
% circularBoundary Determine valid checkerboard corners based on the number
% of zero crossings around the detected corners

% Copyright 2020 The MathWorks, Inc.

% References:
% -----------
% Yunsu Bok, Hyowon Ha, In SoKweon, "Automated checkerboard detection and
% indexing using circular boundaries". Pattern Recognition Letters, Volume
% 73, 1 April 2016.

%#codegen

radius = 5;
thetaResolution = linspace(0,2*pi,36);

circleCoords = radius * [cos(thetaResolution)' sin(thetaResolution)'];
isValidPoint = false(size(imagePoints,1), 1);

imageSize = size(I);
imageWidth = imageSize(2);
imageHeight = imageSize(1);

for i = 1:size(imagePoints,1)

    % Boundary coordinates
    xBoundary = round(imagePoints(i,1) - circleCoords(:,1));
    yBoundary = round(imagePoints(i,2) - circleCoords(:,2));

    xBoundary = min(xBoundary, imageWidth);
    xBoundary = max(xBoundary, 1);

    yBoundary = min(yBoundary, imageHeight);
    yBoundary = max(yBoundary, 1);

    boundaryVector = I(sub2ind(imageSize, yBoundary, xBoundary));

    % Determine the num of sign changes
    boundaryVector = normalize(boundaryVector, 'center');
    signs = sign(boundaryVector);
    diffVec = diff(signs);

    isValidPoint(i) = nnz(abs(diffVec)) > 2;

end

validPoints = imagePoints(isValidPoint, :);
points = zeros(0,2,'like',imagePoints);

% Remove duplicates
while ~isempty(validPoints)

    currentPoint = validPoints(1,:);
    pointDiff = bsxfun(@minus, validPoints, currentPoint);
    dists = hypot(pointDiff(:,1), pointDiff(:,2));

    idx = dists < 5; % Average out corners in neighborhood
    points = [points; round(mean(validPoints(idx,:), 1))]; %#ok<AGROW>
    validPoints(idx, :) = [];

end

end
