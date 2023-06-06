%% 使用同事汪智提供的图片做相机标定，此脚本能够检测到的是圆形标定物，但检测精度低于棋盘亚像素角点检测，估暂停使用
dataRoot = "/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗";
calibrateData = readtable(fullfile(dataRoot,"30度内参标定数据.csv"),VariableNamingRule="preserve");
addpath("utils")

%% the 14th camera
imds = imageDatastore(fullfile(dataRoot,"data/14/"),IncludeSubfolders=false);

patternDims = [11,15];
for i = 1:length(imds.Files)
    image = imread(imds.Files{i});
    [imagePoints,imagesUsed] = detectCirclePoints(image,patternDims);
    if imagesUsed
        J = insertText(image,imagePoints,1:size(imagePoints,1));
        J = insertMarker(J,imagePoints,"x",Color="green",Size=5);
        figure;imshow(J)
        title("Detected a Circle Grid of Dimensions" + mat2str(patternDims))
    end
end

centerDistance = 10;
worldPoints = generateCircleGridPoints(patternDims,centerDistance,...
                        'PatternType','symmetric');
