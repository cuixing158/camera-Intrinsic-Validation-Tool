function [imagePoints,imagesUsed] = detectCirclePoints(image,patternDims,options)
% Brief: 针对同事汪智提供的标定图像(3840*2160*3)高分辨率图像进行圆点检测,镜像对称的圆点，黑白间隔的圆
% Details:
%    本函数设计的目的只针对同事汪智提供的标定图像有效，类似于内建函数detectCircleGridPoints功能
% 
% Syntax:  
%     [imagePoints,imagesUsed] = detectCirclePoints(image)
% 
% Inputs:
%    image - [m,n] size,[uint8,double,single] type,汪智提供的特定标定图像
%    patternDims - [1,2] size, [double] type，模式维度，形如[rows,cols]
%    options - options可选参数，PatternType和CircleColor两个键值
% 
% Outputs:
%    imagePoints - [M,2] size,[double] type,像点坐标
%    imagesUsed - [1,1] size,[logical] type,是否该图像image检测到有效像素点
% 
% Example: 
%    image = imread("/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/内参标定五颗/data/14/1.bmp");
%    patternDims = [11,15];
%    [imagePoints,imagesUsed] = detectCirclePoints(image,patternDims);
%    if imagesUsed
%       J = insertText(image,imagePoints,1:size(imagePoints,1));
%       J = insertMarker(J,imagePoints,"x",Color="green",Size=5);
%       figure;imshow(J)
%       title("Detected a Circle Grid of Dimensions" + mat2str(patternDims))
%    end
%
% See also: detectCircleGridPoints

% Author:                          cuixingxing
% Email:                           xingxing.cui@long-horn.com
% Created:                         26-May-2023 14:40:18
% Version history revision notes:
%                                  None
% Implementation In Matlab R2023a
% Copyright © 2023 long-horn.All Rights Reserved.
%
arguments
    image 
    patternDims (1,2) double % 等同于detectCircleGridPoints函数的参数patternDims
    options.PatternType (1,:) char {mustBeMember(options.PatternType,...
        {'symmetric','asymmetric'})}= 'symmetric'
    options.CircleColor (1,:) char {mustBeMember(options.CircleColor,...
        {'white','black'})}= 'white'
end

grayImg = im2gray(image);
bw = imbinarize(grayImg);

% find white circle
bw1 = imdilate(bw,strel('disk',1));% 连通方块顶角区域，其可能微弱存在非8连通域也无法连通(标定板倾斜)
bw1 = filterCircle(bw1);
% figure;imshow(bw1)

% find black circle
bw2 = ~bw;
se = strel('disk',3);
bw2 = imclose(bw2,se);% 滤除正中间的那个方格的标志 
bw2 = filterCircle(bw2);
% figure;imshow(bw2)


%% 正式检测标定板上的圆
processGrayImage = im2uint8(bw1+bw2);
% patternDims = [11,15];
% figure;imshow(processGrayImage)
[imagePoints,imagesUsed] = detectCircleGridPoints(processGrayImage,patternDims,...
    PatternType=options.PatternType,CircleColor=options.CircleColor);
end

function bw = filterCircle(bw)
% bw image white blob area greater than 5000 set to 0 
CC = bwconncomp(bw,8);
numPixels = cellfun(@numel,CC.PixelIdxList);
idxs = find(numPixels>5000);% 5000大概是标定板上一个小方格的面积
for i = 1:numel(idxs)
    bw(CC.PixelIdxList{idxs(i)}) = 0;
end
end
