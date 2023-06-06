function orderedPts = sortCheckerboardPts(imgPts)
% Brief: 只对同事汪智提供的三角验证图像有效，验证的图像是2*4棋盘格点
% Details:
%    None
% 
% Syntax:  
%     orderedPts = sortCheckerboardPts(imgPts)
% 
% Inputs:
%    imgPts - [8,2] size,[double] type,检测到的棋盘角点亚像素坐标,任意顺序
% 
% Outputs:
%    orderedPts - [8,2] size,[double] type,在图像上从上到下，从左到右排序的亚像素坐标
% 
% Example: 
%    None
% 
% See also: None

% Author:                          cuixingxing
% Email:                           xingxing.cui@long-horn.com
% Created:                         30-May-2023 08:22:29
% Version history revision notes:
%                                  None
% Implementation In Matlab R2023a
% Copyright © 2023 long-horn.All Rights Reserved.
%

assert(size(imgPts,1)==2*4);
[rectx,recty] = minboundrect(imgPts(:,1),imgPts(:,2));
rectx(end) = [];% ensure rectx is 4*1 size
recty(end) = [];% ensure recty is 4*1 size

dist1 = sqrt((rectx(1)-rectx(2)).^2+(recty(1)-recty(2)).^2);
dist2 = sqrt((rectx(2)-rectx(3)).^2+(recty(2)-recty(3)).^2);

%% 寻找最小包围旋转矩形的长边，然后使得其转换成水平方向
if dist1>dist2
    rectx = circshift(rectx,1);
    recty = circshift(recty,1);
end

alpha = atan2d(recty(3)-recty(2),rectx(3)-rectx(2));
rotM3d = rotz(-alpha);
rotM2d = rotM3d(1:2,1:2);
rectanglePts = rotM2d*imgPts';
rectanglePts = rectanglePts';% ensure is M*2 size

%% 在图像上坐标从上到下，从左到右排序
[A,idx] = sortrows(rectanglePts,1,"ascend");
for i = 1:4
    n1 = 2*i-1;
    n2 = 2*i;
    if A(n1,2)>A(n2,2)
        idx([n1,n2]) = idx([n2,n1]);
    end
end
orderedPts = imgPts(idx,:);
end

