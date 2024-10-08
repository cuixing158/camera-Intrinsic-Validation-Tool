% Auto-generated by cameraCalibrator app on 01-Jun-2023
%-------------------------------------------------------


% Define images to process
imageFileNames = {'/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/1.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/10.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/11.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/12.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/13.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/14.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/15.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/16.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/17.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/18.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/19.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/2.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/20.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/21.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/22.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/23.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/24.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/25.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/26.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/27.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/3.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/4.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/5.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/6.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/7.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/8.bmp',...
    '/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/9.bmp',...
    };
% Detect calibration pattern in images
detector = vision.calibration.monocular.CheckerboardDetector();
[imagePoints, imagesUsed] = detectPatternPoints(detector, imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);

% Generate world coordinates for the planar pattern keypoints
squareSize = 100;  % in units of 'millimeters'
worldPoints = generateWorldPoints(detector, 'SquareSize', squareSize);

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', true, ...
    'NumRadialDistortionCoefficients', 3, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [7353.8 0 0;0 7353.1 0;1922.4 1065.5 1], 'InitialRadialDistortion', [-0.16 -1.97 7.83], ...
    'ImageSize', [mrows, ncols]);

% View reprojection errors
h1=figure(Name="repro errors"); showReprojectionErrors(cameraParams);

% Visualize pattern locations
h2=figure(Name="pattern loc"); showExtrinsics(cameraParams, 'CameraCentric');

% Display parameter estimation errors,输出到屏幕显示
displayErrors(estimationErrors, cameraParams);

% For example, you can use the calibration data to remove effects of lens distortion.
undistortedImage = undistortImage(originalImage, cameraParams);


