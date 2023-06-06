%#codegen

classdef Checkerboard < handle
%

%   Copyright 2013-2022 The MathWorks, Inc.

    properties(GetAccess=public, SetAccess=public)
        isValid = false;
        Energy = single(inf);
        BoardCoords;
        BoardIdx;
    end
    
    properties(Access=private)
        Points;
        IsDirectionBad = false(1, 4);
        LastExpandDirection = 1; %'up'
        PreviousEnergy = single(inf);
        IsDistortionHigh = false;
        WarningCleanups;
    end
    
    methods
        function this = Checkerboard(varargin)
            this.BoardIdx = 0;
            this.BoardIdx = zeros(3, 3);
            this.BoardCoords = 0;
            this.BoardCoords = zeros(3, 3, 2);
            this.Points = single(0);
            this.Points = zeros(0,2,'single');
            
            if nargin > 0
                this.IsDistortionHigh = varargin{1};
            end

            persistent warnState
            if this.IsDistortionHigh && isempty(coder.target)
                % Save current state
                if isempty(warnState)
                    warnState = warning;
                end
                
                % Restore warning states on cleanup
                this.WarningCleanups = onCleanup(@()warning(warnState));
                
                % Turn off warnings
                warning('off','MATLAB:polyfit:PolyNotUnique');
                warning('off','MATLAB:polyfit:RepeatedPointsOrRescale');
            end
        end
        
        function initialize(this, seedIdx, points, v1, v2)
            % Constructor. Creates a 4x4 checkerboard with the seed point
            % in the center. points is an Mx2 matrix of x,y coordinates of
            % all possible checkerboard corners. seedIdx is the index of
            % the seed point. The coordinates of the seed point are
            % points(seedIdx, :). e1 and e2 are 2-element vectors
            % specifying the edge orientations at the seed point.
            
            % Inline disabled for faster compile times on larger images (g2711672).
            coder.inline('Never')
            this.BoardIdx = 0;
            this.BoardIdx = zeros(3, 3);
            this.IsDirectionBad = false(1, 4);
            this.BoardCoords = 0;
            this.BoardCoords = zeros(3, 3, 2);
            this.Points = points;
            center = this.Points(seedIdx, :);
            this.BoardIdx(2, 2) = seedIdx;
            this.BoardCoords(2, 2, :) = center;
            this.LastExpandDirection = 1; %'up';
            this.PreviousEnergy = single(inf);
            this.isValid = false;
            
            % compute distances from all the points to the center
            pointVectors = bsxfun(@minus, this.Points, center);
            euclideanDists = hypot(pointVectors(:, 1), pointVectors(:, 2));
            
            % find vertical and horizontal neighbors
            [this.BoardIdx(2, 3)] = findNeighbor(this, pointVectors, euclideanDists, v1);
            [this.BoardIdx(2, 1)] = findNeighbor(this, pointVectors, euclideanDists, -v1);
            [this.BoardIdx(3, 2)] = findNeighbor(this, pointVectors, euclideanDists, v2);
            [this.BoardIdx(1, 2)] = findNeighbor(this, pointVectors, euclideanDists, -v2);
            
            if any(this.BoardIdx(:) < 0)
                this.isValid = false;
                return;
            end
            
            r = this.Points(this.BoardIdx(2, 3), :);
            this.BoardCoords(2, 3, :) = r;
            l = this.Points(this.BoardIdx(2, 1), :);
            this.BoardCoords(2, 1, :) = l;
            d = this.Points(this.BoardIdx(3, 2), :);
            this.BoardCoords(3, 2, :) = d;
            u = this.Points(this.BoardIdx(1,2), :);
            this.BoardCoords(1, 2, :) = u;
            
            % find diagonal neighbors
            up    = u - center;
            down  = d - center;
            right = r - center;
            left  = l - center;
            
            [this.BoardIdx(1, 1)] = findNeighbor(this, pointVectors, euclideanDists, up + left);
            [this.BoardIdx(3, 1)] = findNeighbor(this, pointVectors, euclideanDists, down + left);
            [this.BoardIdx(3, 3)] = findNeighbor(this, pointVectors, euclideanDists, down + right);
            [this.BoardIdx(1, 3)] = findNeighbor(this, pointVectors, euclideanDists, up + right);
            this.isValid = all(this.BoardIdx(:) > 0);
            if ~this.isValid
                return;
            end
            
            this.BoardCoords(1, 1, :) = this.Points(this.BoardIdx(1, 1), :);
            this.BoardCoords(3, 1, :) = this.Points(this.BoardIdx(3, 1), :);
            this.BoardCoords(3, 3, :) = this.Points(this.BoardIdx(3, 3), :);
            this.BoardCoords(1, 3, :) = this.Points(this.BoardIdx(1, 3), :);
            
            this.Energy = computeInitialEnergy(this);
            % A perfect initial board (2x2) should have the energy of -9.
            % This happens when the board is a square grid with no
            % projective or affine distortion.
            if this.IsDistortionHigh
                % Allow lower threshold for high lens distortion which
                % results in a non-linear relationship between the corner
                % locations in the board.
                maxEnergy = -5;
            else
                maxEnergy = -7;
            end
            this.isValid = this.Energy < maxEnergy;
        end
    end
    
    methods(Access=private)
        %------------------------------------------------------------------
        function e = computeInitialEnergy(this)
        % Inline disabled for faster compile times on larger images (g2711672).
            coder.inline('Never')
            if any(this.BoardIdx(:) < 0)
                e = single(inf);
                return;
            end
            
            e = single(0);
            
            % compute energy over rows
            row1 = this.getPoints(1, 1:3);
            row2 = this.getPoints(2, 1:3);
            row3 = this.getPoints(3, 1:3);
            
            num = row1 + row3 - 2 * row2;
            denom = row1 - row3;
            e = max(e, max(hypot(num(:, 1), num(:, 2)) ./ hypot(denom(:, 1), denom(:, 2))));
            
            % compute energy over columns
            col1 = this.getPoints(1:3, 1);
            col2 = this.getPoints(1:3, 2);
            col3 = this.getPoints(1:3, 3);
            
            num = col1 + col3 - 2 * col2;
            denom = col1 - col3;
            e = max(e, max(hypot(num(:, 1), num(:, 2)) ./ hypot(denom(:, 1), denom(:, 2))));
            
            boardSize = single(numel(this.BoardIdx));
            e = boardSize * e - boardSize;
        end
    end
    
    methods
        %------------------------------------------------------------------
        function this = expandBoardFully(this)
            % expands the board as far as possible
            if ~this.isValid
                return;
            end
            
            hasExpanded = true;
            isPartial = false;
            while hasExpanded
                hasExpanded = this.expandBoardOnce(isPartial);
            end
        end
        
        %------------------------------------------------------------------
        function expandPartialBoard(this)
            % Reset IsDirectionBad for partial expansion
            this.IsDirectionBad = false(1,4);
            
            hasExpanded = true;
            isPartial = true;
            while hasExpanded
                hasExpanded = this.expandBoardOnce(isPartial);
            end
            
        end
        
        %------------------------------------------------------------------
        function plot(this)
            % plot the detected checkerboard points
            idx = this.BoardIdx';
            idx = idx(idx > 0);
            points = this.Points(idx, :);
            plot(points(:, 1), points(:, 2), 'r*-'); hold on;
            text(this.BoardCoords(1, 1, 1), this.BoardCoords(1, 1, 2), '(0,0)', 'Color', [1 0 0]);
        end
        
        %------------------------------------------------------------------
        function [points, boardSize] = toPoints(this)
            % returns the points as an Mx2 matrix of x,y coordinates, and
            % the size of the board
            
            if any(this.BoardIdx(:) == 0)
                points = [];
                boardSize = [0 0];
                return;
            end
            
            numPoints = size(this.BoardCoords, 1) * size(this.BoardCoords, 2);
            points = zeros(numPoints, 2);
            x = this.BoardCoords(:, :, 1)';
            points(:, 1) = x(:);
            y = this.BoardCoords(:, :, 2)';
            points(:, 2) = y(:);
            boardSize = [size(this.BoardCoords, 2)+1, size(this.BoardCoords, 1)+1];
        end
    end
    
    methods(Access=private)
        %------------------------------------------------------------------
        function neighborIdx = findNeighbor(this, pointVectors, euclideanDists, v)
            % find the nearest neighbor point in the direction of vector v
            
            % compute normalized dot products
            angleCosines = pointVectors * v' ./ (euclideanDists * hypot(v(1), v(2)));
            
            % dists is a linear combination of euclidean distances and
            % "directional distances"
            dists = euclideanDists + 1.5 * euclideanDists .* (1 - angleCosines);
            
            % eliminate points already in the board
            dists(this.BoardIdx(this.BoardIdx > 0)) = inf;
            
            % eliminate points "behind" the center
            dists(angleCosines < 0) = inf;
            
            % find the nearest neighbor
            [dirDist, neighborIdx] = min(dists);
            if isinf(dirDist)
                neighborIdx = -1;
            end
        end
        
        %------------------------------------------------------------------
        function p = getPoints(this, i, j)
            p = single(this.Points(this.BoardIdx(i, j), :));
        end
        
        %------------------------------------------------------------------
        function success = expandBoardOnce(this, isPartial)
            %directions = {'up', 'down', 'left', 'right'};
            %directions = [1 2 3 4];
            % Inline disabled for faster compile times on larger images (g2711672).
            coder.inline('Never')
            this.PreviousEnergy = this.Energy;
            for i = 1:4
                if ~this.IsDirectionBad(i)
                    this.LastExpandDirection = i;
                    expandBoardDirectionally(this, i, isPartial);
                    if this.Energy < this.PreviousEnergy
                        success = true;
                        return;
                    else
                        this.undoLastExpansion();
                        this.IsDirectionBad(i) = true;
                    end
                end
            end
            success = false;
        end
        
        %------------------------------------------------------------------
        function undoLastExpansion(this)
            this.Energy = this.PreviousEnergy;
            switch this.LastExpandDirection
                case 1 %'up'
                    this.BoardIdx = this.BoardIdx(2:end, :);
                    this.BoardCoords = this.BoardCoords(2:end, :, :);
                    
                case 2 %'down'
                    this.BoardIdx = this.BoardIdx(1:end-1, :);
                    this.BoardCoords = this.BoardCoords(1:end-1, :, :);
                    
                case 3 %'left'
                    this.BoardIdx = this.BoardIdx(:, 2:end);
                    this.BoardCoords = this.BoardCoords(:, 2:end, :);
                    
                case 4 %'right'
                    this.BoardIdx = this.BoardIdx(:, 1:end-1);
                    this.BoardCoords = this.BoardCoords(:, 1:end-1, :);
            end
        end
        
        %------------------------------------------------------------------
        function expandBoardDirectionally(this, direction, isPartial)
            oldEnergy = (this.Energy + numel(this.BoardIdx)) / numel(this.BoardIdx);
            switch direction
                case 1 %'up'
                    idx = 1:3;
                    if this.IsDistortionHigh % Use a curve fitting approach for images with high distortion
                        newIndices = fitPolynomialIndices(this, idx, isPartial, true);
                        if ~all(newIndices) && ~isPartial % For failed detections, use linear prediction workflow
                            predictedPoints = predictPointsVertical(this, idx, isPartial);
                            newIndicesLinear = findClosestIndices(this, predictedPoints, isPartial);

                            failIdx = newIndices == 0;
                            newIndices(failIdx) = newIndicesLinear(failIdx);
                        end
                    else
                        predictedPoints = predictPointsVertical(this, idx, isPartial);
                        newIndices = findClosestIndices(this, predictedPoints, isPartial);
                    end
                    [this.BoardIdx, this.BoardCoords] = expandBoardUp(this, newIndices);
                    newEnergy = computeNewEnergyVertical(this, idx, oldEnergy);
                    
                case 2 %'down'
                    numRows = size(this.BoardCoords, 1);
                    idx = numRows:-1:numRows-2;
                    if this.IsDistortionHigh
                        newIndices = fitPolynomialIndices(this, idx, isPartial, true);
                        if ~all(newIndices) && ~isPartial
                            predictedPoints = predictPointsVertical(this, idx, isPartial);
                            newIndicesLinear = findClosestIndices(this, predictedPoints, isPartial);
                            
                            failIdx = newIndices == 0;
                            newIndices(failIdx) = newIndicesLinear(failIdx);
                        end
                    else
                        predictedPoints = predictPointsVertical(this, idx, isPartial);
                        newIndices = findClosestIndices(this, predictedPoints, isPartial);
                    end
                    [this.BoardIdx, this.BoardCoords] = expandBoardDown(this, newIndices);
                    idx = idx + 1;
                    newEnergy = computeNewEnergyVertical(this, idx, oldEnergy);
                    
                case 3 %'left'
                    idx = 1:3;
                    if this.IsDistortionHigh
                        newIndices = fitPolynomialIndices(this, idx, isPartial, false);
                        if ~all(newIndices) && ~isPartial
                            predictedPoints = predictPointsHorizontal(this, idx, isPartial);
                            newIndicesLinear = findClosestIndices(this, predictedPoints, isPartial);
                            
                            failIdx = newIndices == 0;
                            newIndices(failIdx) = newIndicesLinear(failIdx);
                        end
                    else
                        predictedPoints = predictPointsHorizontal(this, idx, isPartial);
                        newIndices = findClosestIndices(this, predictedPoints, isPartial);
                    end
                    [this.BoardIdx, this.BoardCoords] = expandBoardLeft(this, newIndices);
                    newEnergy = computeNewEnergyHorizontal(this, idx, oldEnergy);
                    
                case 4 %'right'
                    numCols = size(this.BoardCoords, 2);
                    idx = numCols:-1:numCols-2;
                    if this.IsDistortionHigh
                        newIndices = fitPolynomialIndices(this, idx, isPartial, false);
                        if ~all(newIndices) && ~isPartial
                            predictedPoints = predictPointsHorizontal(this, idx, isPartial);
                            newIndicesLinear = findClosestIndices(this, predictedPoints, isPartial);
                            
                            failIdx = newIndices == 0;
                            newIndices(failIdx) = newIndicesLinear(failIdx);
                        end
                    else
                        predictedPoints = predictPointsHorizontal(this, idx, isPartial);
                        newIndices = findClosestIndices(this, predictedPoints, isPartial);
                    end
                    [this.BoardIdx, this.BoardCoords] = expandBoardRight(this, newIndices);
                    idx = idx + 1;
                    newEnergy = computeNewEnergyHorizontal(this, idx, oldEnergy);
                otherwise
                    newEnergy = single(inf);
                    
            end
            
            this.Energy = newEnergy;
        end
        
        %------------------------------------------------------------------
        function newPoints = predictPointsVertical(this, idx, isPartial)
            p1 = squeeze(this.BoardCoords(idx(2), :, :));
            p2 = squeeze(this.BoardCoords(idx(1), :, :));
            newPoints = p2 + p2 - p1;
            
            % During partial detection, prevent expansion along columns with
            % empty points
            if isPartial
                invalidPts = (p1(:,1) == 0) | (p2(:,1) == 0);
                newPoints(invalidPts, :) = NaN;
            end
        end
        
        %------------------------------------------------------------------
        function newPoints = predictPointsHorizontal(this, idx, isPartial)
            p1 = squeeze(this.BoardCoords(:, idx(2), :));
            p2 = squeeze(this.BoardCoords(:, idx(1), :));
            newPoints = p2 + p2 - p1;
            
            % During partial detection, prevent expansion along rows with
            % empty points
            if isPartial
                invalidPts = (p1(:,1) == 0) | (p2(:,1) == 0);
                newPoints(invalidPts, :) = NaN;
            end
        end
        
        %------------------------------------------------------------------
        function newIndices = fitPolynomialIndices(this, idx, isPartial, isVertical)
            % Use a curve-fitting appoach to determine the new corners
            % for expanding the checkerboard. Steps involved:
            % 1. Fit a polynomial curve to each row/col in the current
            %    board.
            % 2. Traverse along each curve in the direction of expansion
            %    incrementally and search for krypoints in a predefined
            %    neighborhood.
            % 3. Return the corners that are closest to the curve.
            
            % Determine independent variable t, for polynomial p(t)
            coordsToUse = findIndependentVar(this, idx, isVertical);
            
            if isVertical
                newIndices = zeros(1,size(this.BoardCoords, 2));
            else
                newIndices = zeros(1,size(this.BoardCoords, 1));
            end

            removedIdx = zeros(1,0);
            coder.varsize('removedIdx', [1 inf]);
            
            % Determine new indices to grow board
            for j = 1:numel(newIndices)
                
                % Find valid indices to determine search radius
                if isVertical
                    validIdx = find(this.BoardCoords(:,j,coordsToUse(1)));
                else
                    validIdx = find(this.BoardCoords(j,:,coordsToUse(1))); 
                end
                
                if numel(validIdx) < 2
                    continue;
                end
                
                % Determine search parameters
                [coordDist, moveDistMultiplier, firstValidIdx] = findSearchParams(this, idx, validIdx, ...
                    j, coordsToUse, isVertical);
                
                % Fit polynomial curve
                currCurve = fitPolyCurve(this, validIdx, j, coordsToUse, isVertical);
                
                % Neighborhood radius
                currRad = coordDist/4; % sign determines direction of motion along curve
                
                % Get reference coordinates to perform search
                if isVertical
                    refCoordValue = this.BoardCoords(firstValidIdx,j,coordsToUse(1));
                else
                    refCoordValue = this.BoardCoords(j,firstValidIdx,coordsToUse(1));
                end 
                currCoord = currRad + refCoordValue;
                
                % Search in circular neighborhood along curve
                searchThreshold = 1.5;
                while (abs(currCoord - refCoordValue) < moveDistMultiplier * searchThreshold * abs(coordDist))
                    if isequal(coordsToUse,[1, 2])
                        currPt = [currCoord, polyval(currCurve, currCoord)];
                    else
                        currPt = [polyval(currCurve, currCoord), currCoord];
                    end
                    
                    % Find nearest neighbor along curve
                    index = findClosestOnCurve(this, currPt, abs(currRad), currCurve, coordsToUse, removedIdx);
                    if ~isempty(index)
                        newIndices(j) = index;
                        removedIdx = [removedIdx index]; %#ok<AGROW>
                        break;
                    end
                    
                    currCoord = currCoord + currRad;
                end
            end
            
            % Reduce the chances of false positive detections of corners
            if nnz(newIndices) < 4 && isPartial
                newIndices(newIndices > 0) = 0;
            end
        end
        
        %------------------------------------------------------------------
        function coordsToUse = findIndependentVar(this, idx, isVertical)
            % Find independent variable for polynomial curve fitting for
            % current board
            
            % Get sample points along rows/columns of checkerboard based on
            % expansion direction
            if isVertical
                nonZeroIdx = this.BoardIdx(idx(1),:) > 0 & this.BoardIdx(idx(2),:) > 0;
                xRangeSample = abs(mean(this.BoardCoords(idx(2),nonZeroIdx,1) - this.BoardCoords(idx(1),nonZeroIdx,1)));
                yRangeSample = abs(mean(this.BoardCoords(idx(2),nonZeroIdx,2) - this.BoardCoords(idx(1),nonZeroIdx,2)));
            else
                nonZeroIdx = this.BoardIdx(:,idx(1)) > 0 & this.BoardIdx(:,idx(2)) > 0;
                xRangeSample = abs(mean(this.BoardCoords(nonZeroIdx,idx(2),1) - this.BoardCoords(nonZeroIdx,idx(1),1)));
                yRangeSample = abs(mean(this.BoardCoords(nonZeroIdx,idx(2),2) - this.BoardCoords(nonZeroIdx,idx(1),2)));
            end
            
            if xRangeSample > yRangeSample
                coordsToUse = [1, 2]; % use X-coordinate
            else
                coordsToUse = [2, 1]; % use Y-coordinate
            end
            
        end
        
        %------------------------------------------------------------------
        function [coordDist, moveMultiplier, firstValidIdx] = findSearchParams(this, idx, validIdx, ...
                currIdx, coordsToUse, isVertical)
            % Calculate distance between boundary rows/cols on checkerboard
            % and first valid index to start searching from along curve.

            if idx(1) == 1 % Expanding up/left
                if isVertical
                    coordDist = this.BoardCoords(validIdx(1),currIdx,coordsToUse(1)) - ...
                        this.BoardCoords(validIdx(2),currIdx,coordsToUse(1));
                else
                    coordDist = this.BoardCoords(currIdx,validIdx(1),coordsToUse(1)) - ...
                        this.BoardCoords(currIdx,validIdx(2),coordsToUse(1));
                end
                moveMultiplier = validIdx(1);
                firstValidIdx = moveMultiplier;
                
                % Scale distance metric based on location of valid
                % indcies in board
                coordDist = coordDist / (validIdx(2) - validIdx(1));
            else  % Expanding down/right
                if isVertical
                    coordDist = this.BoardCoords(validIdx(end),currIdx,coordsToUse(1)) - ...
                        this.BoardCoords(validIdx(end - 1),currIdx,coordsToUse(1));
                    
                    moveMultiplier = size(this.BoardCoords,1) - validIdx(end) + 1;
                else
                    coordDist = this.BoardCoords(currIdx,validIdx(end),coordsToUse(1)) - ...
                        this.BoardCoords(currIdx,validIdx(end - 1),coordsToUse(1));
                    
                    moveMultiplier = size(this.BoardCoords,2) - validIdx(end) + 1;
                end
                firstValidIdx = validIdx(end);
                
                % Scale distance metric based on location of valid
                % indcies in board
                coordDist = coordDist / (validIdx(end) - validIdx(end - 1));
            end
            
        end
        
        %------------------------------------------------------------------
        function curveCoeff = fitPolyCurve(this, validIdx, currIdx, coordsToUse, isVertical)
            % Fit poly curve to row/col in checkerboard
            
            % Determine polynomial degree based on number of valid points
            if nnz(validIdx) > 5
                polyDeg = 4; 
            else
                polyDeg = 2;
            end
            
            % Fit polynomial curves to rows/cols in the current
            % checkerboard based on expansion direction
            if isVertical
                curveCoeff = polyfit(this.BoardCoords(validIdx, currIdx, coordsToUse(1)), ...
                    this.BoardCoords(validIdx, currIdx, coordsToUse(2)), polyDeg);
            else
                curveCoeff = polyfit(this.BoardCoords(currIdx ,validIdx, coordsToUse(1)), ...
                    this.BoardCoords(currIdx, validIdx, coordsToUse(2)), polyDeg);
            end
            
        end

        %------------------------------------------------------------------
        function indices = findClosestIndices(this, predictedPoints, isPartial)
            % returns indices of points closest to the predicted points
            indices = zeros(1, size(predictedPoints, 1));
            
            % Get indices of corners not in board
            remIdx = setdiff(1:size(this.Points, 1), this.BoardIdx(:), 'stable');
            
             % Return if all the points are already in the board
            if isempty(remIdx)
                return;
            end
            
            validPredictions = find(~isnan(predictedPoints(:,1)));
            for i = 1:size(validPredictions, 1)
                p = predictedPoints(validPredictions(i), :);
                diffs = bsxfun(@minus, this.Points(remIdx,:), p);
                dists = hypot(diffs(:, 1), diffs(:, 2));
                distIdx = ismember(remIdx, indices(indices > 0));
                dists(distIdx) = inf;
                [minDist, minDistIdx] = min(dists);
                
                if ~isPartial
                    indices(validPredictions(i)) = remIdx(minDistIdx);
                else
                    % Find neighborhood radius
                    validBoardIdx = nonzeros(this.BoardIdx(:));
                    diffs = bsxfun(@minus, this.Points(validBoardIdx, :), p);
                    dists = hypot(diffs(:, 1), diffs(:, 2));
                    nhoodRadius = min(dists)/2; % Empirical value

                    % For partial expansion, check in a circular
                    % neighborhood of the predicted point.
                    if minDist < nhoodRadius
                        indices(validPredictions(i)) = remIdx(minDistIdx);
                    end
                end
            end
            
            % Reduce chances of false positive detections of corners
            % which slipped through prior elimination routines
            if isPartial && nnz(indices) < 4
                indices(indices > 0) = 0;
            end
        end
        
        %------------------------------------------------------------------
        function idx = findClosestOnCurve(this, predictedPoint, radius, curve, coordsToUse, removedIdx)
            % returns indices of points closest to the curve in the nhood
            % of predictedPoint

            % Corners not in the current board
            remIdx = setdiff(1:size(this.Points, 1), this.BoardIdx(:), 'stable');
            remIdx = setdiff(remIdx, removedIdx, 'stable');
            
            diffs = bsxfun(@minus, this.Points(remIdx,:), predictedPoint);
            dists = hypot(diffs(:, 1), diffs(:, 2));

            candIdx = dists < radius;

            if nnz(candIdx) > 1 % Use the point closest to curve
                firstCoord = predictedPoint(coordsToUse(1)) - radius:1:predictedPoint(coordsToUse(1)) + radius;
                
                % Discretized points on the curve (used to determine
                % distance of detected corners to the curve)
                if isequal(coordsToUse,[1, 2])
                    dataPts = [firstCoord', polyval(curve, firstCoord)'];
                else
                    dataPts = [polyval(curve, firstCoord)', firstCoord'];
                end
                queryPts = this.Points(remIdx(candIdx), :);
                
                % Normal distance approximation to curve
                numQueryPts = size(queryPts,1);
                numDataPts = size(dataPts,1);
                
                dist = zeros(numQueryPts, 1, 'double');
                for i = 1:numQueryPts
                    currPt = repmat(queryPts(i,:), numDataPts, 1);
                    dist(i) = sqrt(min(sum((dataPts - currPt).^2, 2)));
                end
                [~, idxTemp] = min(dist);
                
                idx = find(candIdx, idxTemp);
                idx = remIdx(idx(end));
            elseif nnz(candIdx) == 1
                idx = remIdx(candIdx);
            else
                idx = [];
            end
        end

        %------------------------------------------------------------------
        function [newBoard, newBoardCoords] = expandBoardUp(this, indices)
            newBoard = zeros(size(this.BoardIdx, 1)+1, size(this.BoardIdx, 2));
            newBoard(1, :) = indices;
            newBoard(2:end, :) = this.BoardIdx;
            
            newBoardCoords = zeros(size(this.BoardCoords, 1)+1, ...
                size(this.BoardCoords, 2), size(this.BoardCoords, 3));
            
            validIdx = indices > 0;
            newBoardCoords(1, validIdx, :) = this.Points(indices(validIdx), :);
            
            newBoardCoords(2:end, :, :) = this.BoardCoords;
        end
        
        %------------------------------------------------------------------
        function [newBoard, newBoardCoords] = expandBoardDown(this, indices)
            newBoard = zeros(size(this.BoardIdx, 1)+1, size(this.BoardIdx, 2));
            newBoard(end, :) = indices;
            newBoard(1:end-1, :) = this.BoardIdx;
            
            newBoardCoords = zeros(size(this.BoardCoords, 1)+1, ...
                size(this.BoardCoords, 2), size(this.BoardCoords, 3));
            
            validIdx = indices > 0;
            newBoardCoords(end, validIdx, :) = this.Points(indices(validIdx), :);
            
            newBoardCoords(1:end-1, :, :) = this.BoardCoords;
        end
        
        %------------------------------------------------------------------
        function [newBoard, newBoardCoords] = expandBoardLeft(this, indices)
            newBoard = zeros(size(this.BoardIdx, 1), 1 + size(this.BoardIdx, 2));
            newBoard(:, 1) = indices;
            newBoard(:, 2:end) = this.BoardIdx;
            
            newBoardCoords = zeros(size(this.BoardCoords, 1), ...
                size(this.BoardCoords, 2) + 1, size(this.BoardCoords, 3));

            validIdx = indices > 0;
            newBoardCoords(validIdx, 1, :) = this.Points(indices(validIdx), :);
            
            newBoardCoords(:, 2:end, :) = this.BoardCoords;
        end
        
        %------------------------------------------------------------------
        function [newBoard, newBoardCoords] = expandBoardRight(this, indices)
            newBoard = zeros(size(this.BoardIdx, 1), 1 + size(this.BoardIdx, 2));
            newBoard(:, end) = indices;
            newBoard(:, 1:end-1) = this.BoardIdx;
            
            newBoardCoords = zeros(size(this.BoardCoords, 1), ...
                size(this.BoardCoords, 2) + 1, size(this.BoardCoords, 3));

            validIdx = indices > 0;
            newBoardCoords(validIdx, end, :) = this.Points(indices(validIdx), :);
            
            newBoardCoords(:, 1:end-1, :) = this.BoardCoords;
        end
        
        %------------------------------------------------------------------
        function newEnergy = computeNewEnergyVertical(this, idx, oldEnergy)
            validIdx = this.BoardIdx(idx(1),:) > 0 & this.BoardIdx(idx(2),:) > 0 & ...
                this.BoardIdx(idx(3),:) > 0;
            
            % Compute new energy based on valid triplets (Eq. 7 in Geiger paper)
            newEnergy = single(0);
            if any(validIdx)
                num = squeeze(this.BoardCoords(idx(1),validIdx,:) + this.BoardCoords(idx(3),validIdx,:) ...
                    - 2*this.BoardCoords(idx(2),validIdx,:));
                denom = squeeze(this.BoardCoords(idx(1),validIdx,:) - this.BoardCoords(idx(3),validIdx,:));
                if size(num,2) > 1
                    newEnergy = max(oldEnergy, ...
                        max(hypot(num(:, 1), num(:,2)) ./ hypot(denom(:, 1), denom(:, 2))));
                else
                    newEnergy = max(oldEnergy, ...
                        max(hypot(num(1), num(2)) ./ hypot(denom(1), denom(2))));
                end
            end
            
            % Get valid triplets in checkerboard row
            validPoints = this.BoardIdx(idx(1), :) > 0;
            pattern = [1 1 1];
            validNewRowIdx = this.arrayFind(validPoints, pattern);

            if ~isempty(validNewRowIdx)
                for i = 1:numel(validNewRowIdx)
                    currIdx = validNewRowIdx(i);
                    num = this.BoardCoords(idx(1), currIdx, :) + this.BoardCoords(idx(1), currIdx+2, :)...
                        - 2*this.BoardCoords(idx(1), currIdx+1, :);
                    denom = this.BoardCoords(idx(1), currIdx, :) - this.BoardCoords(idx(1),currIdx+2,:);
                    if newEnergy
                        newEnergy = max(newEnergy, norm(num(:)) ./ norm(denom(:)));
                    else
                        newEnergy = max(oldEnergy, norm(num(:)) ./ norm(denom(:)));
                    end
                end
            end
            if newEnergy
                newEnergy = newEnergy * numel(this.BoardIdx) - numel(this.BoardIdx);
            else
                newEnergy = single(inf);
            end
        end
        
        %------------------------------------------------------------------
        function newEnergy = computeNewEnergyHorizontal(this, idx, oldEnergy)
            validIdx = this.BoardIdx(:,idx(1)) > 0 & this.BoardIdx(:,idx(2)) > 0 & ...
                this.BoardIdx(:,idx(3)) > 0;
            
            % Compute new energy based on valid triplets (Eq. 7 in Geiger paper)
            newEnergy = single(0);
            if any(validIdx)
                num = squeeze(this.BoardCoords(validIdx,idx(1),:) + this.BoardCoords(validIdx,idx(3),:) ...
                    - 2*this.BoardCoords(validIdx,idx(2),:));
                denom = squeeze(this.BoardCoords(validIdx,idx(1),:) - this.BoardCoords(validIdx,idx(3),:));
                if size(num,2) > 1
                    newEnergy = max(oldEnergy, ...
                        max(hypot(num(:, 1), num(:,2)) ./ hypot(denom(:, 1), denom(:, 2))));
                else
                    newEnergy = max(oldEnergy, ...
                        max(hypot(num(1), num(2)) ./ hypot(denom(1), denom(2))));
                end
            end
            
            % Get valid triplets in checkerboard column
            validPoints = (this.BoardIdx(:,idx(1)) > 0)';
            pattern = [1 1 1];
            validNewColIdx = this.arrayFind(validPoints, pattern);

            if ~isempty(validNewColIdx)
                for i = 1:numel(validNewColIdx)
                    currIdx = validNewColIdx(i);
                    num = this.BoardCoords(currIdx, idx(1), :) + this.BoardCoords(currIdx+2, idx(1), :)...
                        - 2*this.BoardCoords(currIdx+1, idx(1), :);
                    denom = this.BoardCoords(currIdx, idx(1), :) - this.BoardCoords(currIdx+2,idx(1),:);
                    if newEnergy
                        newEnergy = max(newEnergy, norm(num(:)) ./ norm(denom(:)));
                    else
                        newEnergy = max(oldEnergy, norm(num(:)) ./ norm(denom(:)));
                    end
                end
            end
            if newEnergy
                newEnergy = newEnergy * numel(this.BoardIdx) - numel(this.BoardIdx);
            else
                newEnergy = single(inf);
            end
        end
        
    end
    
    methods(Static)
        %------------------------------------------------------------------
        function matchedIdx = arrayFind(arr, pattern)
            % Find pattern array locations in another array
            arraySz = numel(arr) - numel(pattern) + 1;
            matchArr = zeros(1, arraySz);
            for idx = 1:arraySz
                matchArr(idx) = all(arr(idx:idx - 1 + numel(pattern)) == pattern);
            end
            matchedIdx = find(matchArr == 1);
        end
    end

end
