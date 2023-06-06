function [imagePoints, boardSize, imageIdx, userCanceled] = detectCheckerboardPointsMine(I, varargin)
% detectCheckerboardPoints Detect a checkerboard pattern in images
%   detectCheckerboardPoints can detect the keypoints of a checkerboard
%   calibration pattern in a single image, a set of images, or stereo image
%   pairs. In order to be detected, the size of the checkerboard must be at
%   least 4-by-4 squares.
%
%   [imagePoints, boardSize] = detectCheckerboardPoints(I) detects a
%   checkerboard in a 2-D truecolor or grayscale image I. imagePoints is an
%   M-by-2 matrix of x-y coordinates of the corners of checkerboard
%   squares. boardSize specifies the checkerboard dimensions as [rows,
%   cols] measured in squares. The number of points M is prod(boardSize-1).
%   If the complete checkerboard cannot be detected, a partially detected
%   checkerboard is returned with [NaN, NaN] as x-y coordinates of missing
%   corners in imagePoints. If a checkerboard cannot be detected,
%   imagePoints = [] and boardSize = [0, 0].
%
%   [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames)
%   detects a checkerboard pattern in images specified by imageFileNames
%   string array. imagePoints is an M-by-2-by-numImages array of x-y coordinates,
%   where numImages is the number of images in which the checkerboard was
%   detected. imagesUsed is a logical vector of the same size as
%   imageFileNames. A value of true indicates that the pattern was detected
%   in the corresponding image.
%
%   [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(images)
%   detects a checkerboard pattern in H-by-W images stored in an
%   H-by-W-by-numColorChannels-by-numImages array.
%
%   [imagePoints, boardSize, pairsUsed] = detectCheckerboardPoints(imageFileNames1,
%   imageFileNames2) detects a checkerboard pattern in stereo pairs of
%   images specified by imageFileNames1 and imageFileNames2 string arrays. A
%   value of true in the logical vector pairsUsed indicates that the
%   checkerboard was detected in the corresponding pair. imagePoints is an
%   M-by-2-by-numPairs-by-numCameras array of x-y coordinates.
%   imagePoints(:,:,:,1) are the points from the first set of images, and
%   imagePoints(:,:,:,2) are the points from the second one. The
%   checkerboard needs to be fully visible in the image pairs for it be
%   detected. Unlike for single camera calibration, partially detected
%   checkerboards are rejected for stereo image pairs.
%
%   [imagePoints, boardSize, pairsUsed] = detectCheckerboardPoints(images1, images2)
%   detects a checkerboard pattern in stereo pairs of H-by-W images stored
%   in H-by-W-by-numColorChannels-by-numImages arrays.
%
%   [...] = detectCheckerboardPoints(..., Name, Value) specifies additional
%   name-value pair arguments described below:
%
%   'MinCornerMetric'       Nonnegative scalar to specify the threshold for
%                           corner metric. If the image is noisy or highly
%                           textured, increase this threshold to reduce the
%                           number of false corners.
%
%                           Default: 0.15 when 'HighDistortion' is false, or
%                                    0.12 when 'HighDistortion' is true
%
%   'HighDistortion'        Set to true when the images contain high level
%                           of distortion that is typical of a wide field
%                           of view camera. When enabled, the algorithm is
%                           more resilient to higher level of image
%                           distortion at the expense of processing speed.
%
%                           Default: false
%
%   'PartialDetections'     Partially detected checkerboards are returned
%                           when set to true, which are otherwise
%                           discarded. Missing keypoint detections are
%                           filled with [NaN, NaN] coordinates. This option
%                           is ignored for stereo image pairs.
%
%                           Default: true
%
% Class Support
% -------------
% I, images, images1, and images2 can be uint8, int16, uint16, single, or double.
% imageFileNames, imageFileNames1, and imageFilenames2 must be cell arrays of
% strings.
%
% Notes
% -----
% - For single camera images, if the complete checkerboard is not detected in
% any of the input images, the largest detected checkerboard is used as the
% reference board, and the size of this board is returned in boardSize.
%
% - For images where the entire checkerboard is not visible, the partially
% detected board is oriented such that the location of the origin and the
% arrangement of the corners is consistent with the completely visible
% checkerboard if possible.
%
% Example 1: Detect a checkerboard in one image
% ---------------------------------------------
% % Load an image containing the checkerboard pattern
% imageFileName = fullfile(toolboxdir('vision'),...
%         'visiondata', 'calibration', 'webcam', 'image4.tif');
% I = imread(imageFileName);
%
% % Detect the checkerboard points
% [imagePoints, boardSize] = detectCheckerboardPoints(I);
%
% % Display detected points
% J = insertText(I, imagePoints, 1:size(imagePoints, 1));
% J = insertMarker(J, imagePoints, 'o', 'Color', 'red', 'Size', 5);
% imshow(J);
% title(sprintf('Detected a %d x %d Checkerboard', boardSize));
%
% Example 2: Detect checkerboard in images with high distortion
% -------------------------------------------------------------
% % Create an imageDatastore of calibration images
% imds = imageDatastore(fullfile(toolboxdir('vision'), 'visiondata', ...
%      'calibration', 'gopro'));
%
% % Detect calibration pattern. 'HighDistortion' option is typically used 
% % with fisheye lens images.
% [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imds.Files(1:4), ...
%     'HighDistortion', true);
%
% % Display detected points
% for i = 1:4
%     % Read image
%     I = readimage(imds, i);
%
%     % Insert markers at detected point locations
%     I = insertMarker(I, imagePoints(:,:,i), 'o', 'Color', 'red', 'Size', 10);
%
%     % Display image
%     subplot(2, 2, i);
%     imshow(I);
% end
%
% Example 3: Detect checkerboard in stereo images
% -----------------------------------------------
% % Specify calibration images
% imageDir = fullfile(toolboxdir('vision'), 'visiondata', ...
%     'calibration', 'stereo');
% leftImages = imageDatastore(fullfile(imageDir, 'left'));
% rightImages = imageDatastore(fullfile(imageDir, 'right'));
% images1 = leftImages.Files;
% images2 = rightImages.Files;
%
% % Detect the checkerboards
% [imagePoints, boardSize, pairsUsed] = detectCheckerboardPoints(images1,...
%   images2);
%
% % Display points from first 4 camera 1 images
% images1 = images1(pairsUsed);
% figure;
% for i = 1:4
%     I = imread(images1{i});
%     I = insertMarker(I, imagePoints(:,:,i,1), 'o', 'Color', 'red', 'Size', 10);
%     subplot(2, 2, i);
%     imshow(I);
% end
% annotation('textbox', [0 0.9 1 0.1], 'String', 'Camera 1', ...
%    'EdgeColor', 'none', ...
%    'HorizontalAlignment', 'center')
%
% % Display points from first 4 camera 2 images
% images2 = images2(pairsUsed);
% figure;
% for i = 1:4
%     I = imread(images2{i});
%     I = insertMarker(I, imagePoints(:,:,i,2), 'o', 'Color', 'red', 'Size', 10);
%     subplot(2, 2, i);
%     imshow(I);
% end
% annotation('textbox', [0 0.9 1 0.1], 'String', 'Camera 2', ...
%    'EdgeColor', 'none', ...
%    'HorizontalAlignment', 'center')
%
% See also estimateCameraParameters, generateCheckerboardPoints,
%   cameraCalibrator, cameraParameters, stereoParameters

% Copyright 2013-2022 The MathWorks, Inc.

% References:
% -----------
% Andreas Geiger, Frank Moosmann, Omer Car, and Bernhard Schuster,
% "Automatic Camera and Range Sensor Calibration using a single Shot.
% In International Conference on Robotics and Automation (ICRA), St. Paul,
% USA, May 2012.

%#codegen
%#ok<*EMCLS>
%#ok<*EMCA>

if isempty(coder.target)
    % Convert strings to chars for simulation

    if isstring(I)
        I = convertStringsToChars(I);
    end

    if nargin>1
        [varargin{:}] = convertStringsToChars(varargin{:});
    end

    [images2, parent, showProgressBar, minCornerMetric, highDistortion, usePartial] = parseInputs(varargin{:});
else
    coder.internal.errorIf(ischar(I), 'vision:calibrate:codegenFileNamesNotSupported');
    coder.internal.errorIf(isstring(I), 'vision:calibrate:codegenFileNamesNotSupported');
    coder.internal.errorIf(iscell(I), 'vision:calibrate:codegenFileNamesNotSupported');
    coder.internal.errorIf(isnumeric(I) && length(size(I)) > 3,...
        'vision:calibrate:codegenMultipleImagesNotSupported');
    [images2, showProgressBar,minCornerMetric, highDistortion, usePartial] = parseInputsCodegen(varargin{:});
    parent = [];
end

if isempty(images2)
    % single camera
    [imagePoints, boardSize, imageIdx, userCanceledTmp, fullBoardDetected] = detectMono(I, parent,...
        showProgressBar, minCornerMetric, highDistortion, usePartial);
else
    % 2-camera stereo
    images1 = I;
    checkStereoImages(images1, images2);
    
    % Partial checkerboards are disabled for stereo images for now. Will be
    % enabled later when the dependent functionalities are available.
    usePartial = false;
    
    [imagePoints, boardSize, imageIdx, userCanceledTmp] = detectStereo(images1, ...
        images2, parent, showProgressBar, minCornerMetric, highDistortion, usePartial);
    
    % Set to true since partial checkerboard detection is disabled for
    % stereo images
    fullBoardDetected = true; 
end

checkThatBoardIsAsymmetric(boardSize, fullBoardDetected);
if showProgressBar
    userCanceled = userCanceledTmp;
else
    userCanceled = false;
end

%--------------------------------------------------------------------------
function [image2, parent, showProgressBar, minCornerMetric, highDistortion, usePartial] = parseInputs(varargin)

% Check if the second argument is the second set of images
% Need to do this "by hand" because inputParser does not
% handle optional string arguments.

isSecondArgumentString =  ~isempty(varargin) && ischar(varargin{1});
isThirdArgumentValue = ~mod(nargin,2);

% pv-pairs come in pairs
isSecondArgumentNameValuePair = isSecondArgumentString && isThirdArgumentValue;

if isempty(varargin) || isSecondArgumentNameValuePair
    image2 = [];
    args = varargin;
else
    image2 = varargin{1};
    if numel(varargin) > 1
        args = varargin(2:end);
    else
        args = {};
    end
end

% Parse the Name-Value pairs
parser = inputParser;
parser.addParameter('ShowProgressBar', false, @checkShowProgressBar);
parser.addParameter('ProgressBarParent', [], @checkProgressBarParent);

cornerMetricDef = 0.15; % when highDistortion is disabled
parser.addParameter('MinCornerMetric', cornerMetricDef, @checkMinCornerMetric);
parser.addParameter('HighDistortion', false, @checkHighDistortion);
parser.addParameter('PartialDetections', true, @checkPartialDetections);
parser.parse(args{:});
showProgressBar = parser.Results.ShowProgressBar;
parent          = parser.Results.ProgressBarParent;
highDistortion  = parser.Results.HighDistortion;
usePartial      = parser.Results.PartialDetections;
minCornerMetric = parser.Results.MinCornerMetric;

if highDistortion && any(strcmp('MinCornerMetric', parser.UsingDefaults))
    minCornerMetric = 0.12; % Allow lower threshold for high distortion images
end
    
%--------------------------------------------------------------------------
function [image2, showProgressBar, minCornerMetric, highDistortion, usePartial] = parseInputsCodegen(varargin)
showProgressBar = false;

isSecondArgumentString =  ~isempty(varargin) && (ischar(varargin{1}) || isstring(varargin{1}));
isThirdArgumentValue = ~mod(nargin,2);

isSecondArgumentNameValuePair = isSecondArgumentString && isThirdArgumentValue;

if isempty(varargin) || isSecondArgumentNameValuePair
    image2 = [];
    args = varargin;
else
    image2 = varargin{1};
    if numel(varargin) > 1
        args = varargin(2:end);
    else
        args = {};
    end
end

if ~isempty(args)
    params = struct( ...
        'MinCornerMetric', uint32(0), ...
        'HighDistortion', false, ...
        'PartialDetections', true);

    popt = struct( ...
        'CaseSensitivity', false, ...
        'StructExpand',    true, ...
        'PartialMatching', true);

    optarg = eml_parse_parameter_inputs(params, popt, args{:});
    highDistortion = eml_get_parameter_value(optarg.HighDistortion, false, args{:});
    usePartial = eml_get_parameter_value(optarg.PartialDetections, true, args{:});

    % Use default value for MinCornerMetric
    if optarg.MinCornerMetric == uint32(0)
        if highDistortion
            minCornerMetric = 0.12;
        else
            minCornerMetric = 0.15;
        end
    else
        minCornerMetric = eml_get_parameter_value(optarg.MinCornerMetric, 0.15, args{:});
    end

    % MinCornerMetric
    checkMinCornerMetric(minCornerMetric);

    % HighDistortion
    checkHighDistortion(highDistortion);
    
    % PartialDetections
    checkPartialDetections(usePartial);
else
    minCornerMetric = 0.15;
    highDistortion = false;
    usePartial = true;
end

%--------------------------------------------------------------------------
% Detect the checkerboards in a single set of images
function [points, boardSize, imageIdx, userCanceled, fullBoardDetected] = ...
    detectMono(I, parent, showProgressBar, minCornerMetric, highDistortion, usePartial)

userCanceled = false;
if iscell(I)
    % detect in a set of images specified by file names
    fileNames = I;
    checkFileNames(fileNames);
    [points, boardSize, imageIdx, userCanceledTmp, fullBoardDetected] = ...
        detectCheckerboardFiles(fileNames, parent, showProgressBar, minCornerMetric, highDistortion, usePartial);
    if showProgressBar
        userCanceled = userCanceledTmp;
    end
elseif ischar(I)
    % detect in a single image specified by a file name
    fileName = I;
    checkFileName(I);
    I = imread(fileName);
    [points, boardSize] = detectCheckerboardInOneImage(I, minCornerMetric, highDistortion, usePartial);
    imageIdx = ~isempty(points);
     
    fullBoardDetected = true;
    if imageIdx
        fullBoardDetected = ~any(isnan(points(:,1)));
    end
elseif ndims(I) > 3
    % detect in a stack of images
    checkImageStack(I);
    [points, boardSize, imageIdx, userCanceledTmp, fullBoardDetected] = ...
        detectCheckerboardStack(I, parent, showProgressBar, minCornerMetric, highDistortion, usePartial);
    if showProgressBar
        userCanceled = userCanceledTmp;
    end
else
    % detect in a single image
    checkImage(I);
    [points, boardSize] = detectCheckerboardInOneImage(I, minCornerMetric, highDistortion, usePartial);
    imageIdx = ~isempty(points);
    
    fullBoardDetected = true;
    if imageIdx
        fullBoardDetected = ~any(isnan(points(:,1)));
    end
end

%--------------------------------------------------------------------------
% Detect the checkerboards in stereo pairs.
function [points, boardSize, imageIdx, userCanceled] = ...
    detectStereo(images1, images2, parent, showProgressBar, minCornerMetric, highDistortion, usePartial)

if isnumeric(images1) && size(images1, 4) == 1 % pair of single images
    [points1, boardSize1] = detectMono(images1, [], false, minCornerMetric, highDistortion, usePartial);
    [points2, boardSize2] = detectMono(images2, [], false, minCornerMetric, highDistortion, usePartial);

    userCanceled = false;
    if ~isequal(boardSize1, boardSize2)
        points = zeros(0, 2);
        boardSize = [0,0];
        imageIdx = false;
    else
        points = cat(4, points1, points2);
        boardSize = boardSize1;
        imageIdx = true;
    end

    if isempty(points)
        imageIdx = false;
    end
else
    % concatenate the two sets of images into one
    images = concatenateImages(images1, images2);

    % detect the checkerboards in the combined set
    [points, boardSize, imageIdx, userCanceled] = detectMono(images, parent,...
        showProgressBar, minCornerMetric, highDistortion, usePartial);

    if userCanceled
        points = zeros(0, 2);
        boardSize = [0,0];
    else
        % separate the points from images1 and images2
        [points, imageIdx] = vision.internal.calibration.separatePoints(points, imageIdx);

        if isempty(points)
            boardSize = [0 0];
        end
    end
end

%--------------------------------------------------------------------------
function images = concatenateImages(images1, images2)
if iscell(images1)
    images = {images1{:}, images2{:}}; %#ok
elseif ischar(images1)
    images = {images1, images2};
else
    images = cat(4, images1, images2);
end

%--------------------------------------------------------------------------
function tf = checkShowProgressBar(showProgressBar)
validateattributes(showProgressBar, {'logical', 'numeric'},...
    {'scalar'}, mfilename, 'ShowProgressBar');
tf = true;

%--------------------------------------------------------------
function tf = checkProgressBarParent(progressBarParent)
if ~isempty(progressBarParent)
    validateattributes(progressBarParent, {'matlab.ui.container.internal.AppContainer'},...
        {}, mfilename, 'ProgressBarParent');
end
tf = true;

%--------------------------------------------------------------------------
function tf = checkMinCornerMetric(value)
validateattributes(value, {'single', 'double'},...
    {'scalar', 'real', 'nonnegative', 'finite'}, mfilename, 'MinCornerMetric');
tf = true;

%--------------------------------------------------------------------------
function tf = checkHighDistortion(highDistortion)
validateattributes(highDistortion, {'logical', 'numeric'},...
    {'scalar','binary'}, mfilename, 'HighDistortion');
tf = true;

%--------------------------------------------------------------------------
function tf = checkPartialDetections(usePartial)
validateattributes(usePartial, {'logical', 'numeric'},...
    {'scalar','binary'}, mfilename, 'PartialDetections');
tf = true;

%--------------------------------------------------------------------------
function checkImage(I)
vision.internal.inputValidation.validateImage(I, 'I');

%--------------------------------------------------------------------------
function checkImageStack(images)
validClasses = {'double', 'single', 'uint8', 'int16', 'uint16'};
validateattributes(images, validClasses,...
    {'nonempty', 'real', 'nonsparse'},...
    mfilename, 'images');
coder.internal.errorIf(size(images, 3) ~= 1 && size(images, 3) ~= 3,...
    'vision:dims:imageNot2DorRGB');

%--------------------------------------------------------------------------
function checkFileNames(fileNames)
validateattributes(fileNames, {'cell'}, {'nonempty', 'vector'}, mfilename, ...
    'imageFileNames');
for i = 1:numel(fileNames)
    checkFileName(fileNames{i});
end

%--------------------------------------------------------------------------
function checkFileName(fileName)
validateattributes(fileName, {'char'}, {'nonempty'}, mfilename, ...
    'elements of imageFileNames');

try %#ok<EMTC>
    state = warning('off','imageio:tifftagsread:badTagValueDivisionByZero');
    imfinfo(fileName);
catch e
    warning(state);
    throwAsCaller(e);
end
warning(state);

%--------------------------------------------------------------------------
function checkStereoImages(images1, images2)
coder.internal.errorIf(strcmp(class(images1), class(images2)) == 0,...
    'vision:calibrate:stereoImagesMustBeSameClass');

coder.internal.errorIf(~ischar(images1) && any(size(images1) ~= size(images2)),...
    'vision:calibrate:stereoImagesMustBeSameSize');

%--------------------------------------------------------------------------
function checkThatBoardIsAsymmetric(boardSize, fullBoardDetected)
% ideally, a board should be asymmetric: one dimension should be even, and
% the other should be odd.
if isempty(coder.target) && fullBoardDetected
    if ~all(boardSize == 0) && (~xor(mod(boardSize(1), 2), mod(boardSize(2), 2))...
            || boardSize(1) == boardSize(2))
        s = warning('query', 'backtrace');
        warning off backtrace;
        warning(message('vision:calibrate:boardShouldBeAsymmetric'));
        warning(s);
    end
end

%--------------------------------------------------------------------------
% Detect checkerboards in a set of images specified by file names
function [points, boardSize, imageIdx, userCanceled, fullBoardDetected] = ...
    detectCheckerboardFiles(fileNames, parent, showProgressBar, minCornerMetric, highDistortion, usePartial)
numImages = numel(fileNames);
boardPoints = cell(1, numImages);
boardSizes = zeros(numImages, 2);
userCanceled = false;
if showProgressBar
    waitBar = createProgressbar(numImages, parent);
end
for i = 1:numImages
    if showProgressBar && waitBar.Canceled
            points = [];
            boardSize = [0 0];
            imageIdx =[];
            userCanceled = true;
            return;
    end

    im = imread(fileNames{i});
    [boardPoints{i}, boardSizes(i,:)] = detectCheckerboardInOneImage(im, minCornerMetric, highDistortion, usePartial);
    if showProgressBar
        waitBar.update();
    end
end
[points, boardSize, imageIdx, fullBoardDetected] = chooseValidBoards(boardPoints, boardSizes, minCornerMetric, ...
    highDistortion, usePartial, fileNames);

%--------------------------------------------------------------------------
% Detect checkerboards in a stack of images
function [points, boardSize, imageIdx, userCanceled, fullBoardDetected] = ...
    detectCheckerboardStack(images, parent, showProgressBar, minCornerMetric, highDistortion, usePartial)
numImages = size(images, 4);
boardPoints = cell(1, numImages);
boardSizes = zeros(numImages, 2);
userCanceled = false;
if showProgressBar
    waitBar = createProgressbar(numImages, parent);
end
for i = 1:numImages
    if showProgressBar && waitBar.Canceled
            points = [];
            boardSize = [0 0];
            imageIdx =[];
            userCanceled = true;
            return;
    end
    im = images(:, :, :, i);
    [boardPoints{i}, boardSizes(i,:)] = detectCheckerboardInOneImage(im, minCornerMetric, highDistortion, usePartial);
    if showProgressBar
        waitBar.update();
    end
end
[points, boardSize, imageIdx, fullBoardDetected] = chooseValidBoards(boardPoints, boardSizes, minCornerMetric, ...
    highDistortion, usePartial, images);

%--------------------------------------------------------------------------
% Determine which board size is the most common in the set.
function [points, boardSize, imageIdx, fullBoardDetected] = chooseValidBoards(boardPoints, boardSizes, ...
    minCornerMetric, highDistortion, usePartial, images)
uniqueBoardIds = 2.^boardSizes(:, 1) .* 3.^boardSizes(:, 2);

% Eliminate images where no board was detected.
% The unique board id in this case is 2^0 + 3^0 = 1.
% Replace all 1's by a sequence of 1:n * 1e10, which will be different from
% all other numbers which are only multiples of 2 and 3.
zeroIdx = (uniqueBoardIds == 1);

% Defaults to true. Set to false when no reference (dominant) board is
% detected.
fullBoardDetected = true;

if all(zeroIdx)
    % When none of images have any boards, return [] for points and [0 0]
    % for the board size. In this case, the input boardPoints is a cell
    % array of empty matrices, which is used to allocate the height of the
    % imageIdx output vector.
    points = [];
    boardSize = [0 0];
    numImages = numel(boardPoints);
    imageIdx = false(numImages,1);
else
    uniqueBoardIds(zeroIdx) = (1:sum(zeroIdx)) * 5;

    % Find the most common value among unique board ids.
    [~, freq, modes] = mode(uniqueBoardIds);
    modeBoardId = max(modes{1});

    if usePartial
        numBoards = size(boardSizes, 1);
        
        % Get min number of same board detections for it to be considered
        % as the reference board
        if numBoards <= 3
            freqThreshold = 2;
        else
            freqThreshold = 3;
        end
        
        if freq >= freqThreshold
            % Use the board corresponding to the mode as the reference board
            refBoardSize = boardSizes(find(uniqueBoardIds == modeBoardId, 1), :);
        else
            % Use the board with the maximum size as the reference board
            fullBoardDetected = false;
            refBoardSize = max(boardSizes, [], 1);
        end
        
        % Accept all non-empty boards which are of the same size or smaller
        % than the reference board along both dimensions
        imageIdx = all(boardSizes <= refBoardSize, 2) & (boardSizes(:,1) > 0);
        
        % Retry detection for failed images without partial boards. At this
        % point, the rejection could be due to the detected board being larger
        % than the reference board along any dimension.
        retryIdx = find(~imageIdx);
        for idx = retryIdx'
            
            if iscell(images)
                im = imread(images{idx});
            else
                im = images(:, :, :, idx);
            end
            
            % Retry detection without looking for partial boards
            usePartial = false;
            [currBoardPoints, currBoardSize] = detectCheckerboardInOneImage(im, minCornerMetric, ...
                highDistortion, usePartial);
            
            if ~isempty(currBoardPoints) && all(currBoardSize <= refBoardSize)
               boardPoints{idx} = currBoardPoints;
               boardSizes(idx,:) = currBoardSize;
               imageIdx(idx) = true;
            end
        end
        
        % Pad smaller boards to be of identical size to the reference board
        boardPoints = padPartialBoards(boardPoints, boardSizes, imageIdx, refBoardSize);
        
        boardSize = refBoardSize;
    else
        imageIdx = (uniqueBoardIds == modeBoardId);
        boardSize = boardSizes(imageIdx, :);
        boardSize = boardSize(1, :);
    end

    % Get the corresponding points
    points = boardPoints(imageIdx);
    points = cat(3, points{:});
end

%--------------------------------------------------------------------------
function boardPoints = padPartialBoards(boardPoints, boardSizes, validBoards, refBoardSize)

validIdx = find(validBoards);
for boardIdx = 1:numel(validIdx)
   
    currBoardSize = boardSizes(validIdx(boardIdx), :);
    
    % Pad zeros assuming the upper left corner point as the origin. If
    % this is missing (not visible/detected), the actual image location of
    % the origin will have to be determined after camera parameter
    % estimation
    padSize = refBoardSize - currBoardSize;
    
    currBoardPoints = boardPoints{validIdx(boardIdx)};
    
    currBoardX = reshape(currBoardPoints(:,1), currBoardSize - 1);
    currBoardX = padarray(currBoardX, padSize, NaN, 'post');
    
    currBoardY = reshape(currBoardPoints(:,2), currBoardSize - 1);
    currBoardY = padarray(currBoardY, padSize, NaN, 'post');

    boardPoints{validIdx(boardIdx)} = [currBoardX(:), currBoardY(:)];

end 

%--------------------------------------------------------------------------
function [points, boardSize] = detectCheckerboardInOneImage(Iin, ...
    minCornerMetric, highDistortion, usePartial)
if ismatrix(Iin)
    Igray = Iin;
else
    Igray = rgb2gray(Iin);
end
I = im2single(Igray);

% Set bandwidth to smooth the image
if highDistortion
    % Use lower standard deviation to reduce smoothing in high distortion
    % images to prevent loss of features at the edges of Field-Of-View.
    sigma = 1.5;
else
    sigma = 2;
end

[points, boardSize] = detectCheckerboard(...
    I, sigma, minCornerMetric, highDistortion, usePartial);

% Use a larger kernel size if no points are detected
if isempty(points)
    sigma = 4;
    [points, boardSize] = detectCheckerboard(...
        I, sigma, minCornerMetric, highDistortion, usePartial);
end

% Replace missing corners with NaNs
if ~isempty(points) && usePartial
    zeroIdx = points(:,1) == 0;
    points(zeroIdx, :) = NaN;
end

%--------------------------------------------------------------------------
function waitBar = createProgressbar(numImages, parent)
    titleId = 'vision:calibrate:AnalyzingImagesTitle';
    messageId = 'vision:calibrate:detectCheckerboardWaitbar';
    tag = 'CheckerboardDetectionProgressBar';
    waitBar = vision.internal.uitools.ProgressBar(numImages, messageId, titleId,...
        tag, parent);
