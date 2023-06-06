function vertices = getVertices(rotatedRect)%#codegen
% 功能：根据一个旋转矩形rotatedRect返回该矩形的4个顶点坐标。功能等同于opencv中
% rotatedRect类的points成员函数。
% 输入：
%     rotatedRect
%       1*5 double,形如[xcenter, ycenter, width, height,
%       yaw]，表示一个旋转矩形,yaw单位为度,以绕x轴逆时针为正
% 输出：
%    vertices
%         4*2 double，旋转矩形的顶点坐标，每行形如[x,y]为一个顶点的坐标，在图像
%                    坐标系下顺序为：左下、左上、右上、右下
% reference:
%   [1] https://en.wikipedia.org/wiki/Rotation_matrix
%   [2] https://blog.csdn.net/Mynameisyournamewuyu/article/details/88650409
%   [3] https://github.com/opencv/opencv/blob/6ae8103022cdb3cd79f417945fd8332c28298b7b/modules/core/src/types.cpp#L173
%
% Email:cuixingxing150@gmail.com
% date:2022.1.20 cuixingxing create this file
% 记录：此函数设计支持高效的C代码生成
%
% Example:
% rotatedRect = [100,100,200,100,-60];% [centerx,centery,width,height,theta]
% vertices = getVertices(rotatedRect);
% 
% pts = [vertices;vertices(1,:)];
% plot(pts(:,1),pts(:,2),pts(1,1),pts(1,2),'ro')
% axis equal
%
cx = rotatedRect(1);
cy = rotatedRect(2);
width = rotatedRect(3);
height = rotatedRect(4);
theta = rotatedRect(5);% 单位：度
vertices = 0.5*[-width*cosd(theta)-height*sind(theta),-width*sind(theta)+height*cosd(theta);
    -width*cosd(theta)+height*sind(theta),-width*sind(theta)-height*cosd(theta);
    width*cosd(theta)+height*sind(theta),width*sind(theta)-height*cosd(theta);
    width*cosd(theta)-height*sind(theta),width*sind(theta)+height*cosd(theta)]+...
    repmat([cx,cy],4,1);
end