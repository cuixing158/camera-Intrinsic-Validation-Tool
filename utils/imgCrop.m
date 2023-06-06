function [croppedImg,tform] = imgCrop(srcImg,rect)
% 功能：build-in中的imcrop增强型函数，额外支持旋转矩形截图
% 输入：
%     srcImg: source image
%     rect: 1*4 double,形如[x,y,width,height]代表垂直坐标轴的矩形或者
%           1*5 double 形如[x,y,width,height,yaw] 代表旋转矩形，yaw为弧度，绕x轴逆时针为正
% 输出：
%     croppedImg: m*n 与srcImg同类型的截取图像
%
% Email: cuixingxing150@gmail.com
% 2022.3.5 create this file
% 2023.5.31 add tform return param
% Impleametation in Matlab 2023a
%
arguments
    srcImg
    rect (1,:) double
end

if length(rect)==4 % vertical rectangle
    croppedImg = imcrop(srcImg,rect);
elseif length(rect)==5 % rotate rectangle
    x = rect(1);
    y = rect(2);
    width = rect(3);
    height = rect(4);
    theta = -rect(5);
    
    % transform, or use the comment section to get 'tform'
    %     rect = [rect(1:4),rect(5)*180/pi];
    %     vertices = getVertices(rect);
    %     matchedPoints1 = vertices;
    %     matchedPoints2 = [1,height;
    %         1,1;
    %         width,1;
    %         width,height];
    %     transformType = 'rigid';
    %     tform = estimateGeometricTransform2D(matchedPoints1,matchedPoints2,transformType);% or use fitgeotrans
    
    % Matlab R2022a or before use follow code
    % rot = [cos(theta),sin(theta);
    %     -sin(theta),cos(theta)];
    % trans = [width,height]/2-[x,y]*rot;
    % tform = rigid2d(rot,trans);
    
    % Matlab R2022b or later prefer use follow code
    rot = [cos(theta),-sin(theta);
         sin(theta),cos(theta)];
    trans = [width,height]'/2-rot*[x,y]';
    tform = rigidtform2d(rot,trans);
    
    outputView = imref2d(round([height,width]));
    croppedImg = imwarp(srcImg,tform,'OutputView',outputView);
else
    croppedImg = [];
end
