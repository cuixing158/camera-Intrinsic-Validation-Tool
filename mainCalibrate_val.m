%% 通过对某个模组相机进行内参"精度"验证，验证方式为：对于已估计好的某个模组相机内参(包括畸变中心和系数)在不同角度拍摄位于同一平面内且标靶中心距为2米的两个标靶视角情况下验证其内参是否能够满足合规性。
%
% 2023.6.1 结论：对两个标靶中心测距，当棋盘格大小为350mm*350mm时候，视角测量距离均在2米左右，可达到毫米级误差，所估的模组相机内参(包括畸变中心和系数)均满足要求！
%

%% step1: 读入相机内参,标靶图片和目标检测器
addpath("buildInRefine");
addpath("utils")
load data/ID14_cameraParams.mat
load data/fasterRcnnDetector.mat
imageFolder = "E:\long_horn_Colleagues\from_wangzhi\三角测距\三角标定图片\14";
imds = imageDatastore(imageFolder);
inputSize = [224,224,3];

%% step2: 检测标靶和验证其中心距是否满足2m要求
for i = 1:numel(imds.Files)
    img = readimage(imds,i);
    [img,origin] = undistortImage(img,cameraParams);

    imgFasterRcnn = im2single(imresize(img,inputSize(1:2)));
    [bboxes, scores, labels] = detect(fasterRcnnDetector,imgFasterRcnn);
    cropImg = img;
    offsetX = 0;
    offsetY = 0;
    if ~isempty(bboxes)
        [~,idx] = maxk(scores,1);
        bbox = bboxes(idx,:);
        bbox = bboxresize(bbox,size(img,[1,2])./inputSize(1:2));
        cropImg = imcrop(img,bbox);
        offsetX = bbox(1);
        offsetY = bbox(2);
    end
    imagePoints = detectCheckerboardPointsMine(cropImg);
    if size(imagePoints,1)==8
        imagePoints = [offsetX+imagePoints(:,1)-1,offsetY+imagePoints(:,2)-1];
    end

    figure;imshow(img);hold on;
    if size(imagePoints,1)~=8
        [centers, radii, metric] = imfindcircles(img,[20 70],ObjectPolarity="dark",Sensitivity=0.5,EdgeThreshold=0.1);
        viscircles(centers, radii,'Color','b');
        if size(centers,1)==2
             centersDist = vecnorm(diff(centers));
             offset = 0.25*centersDist;
              center = mean(centers);
              oriention = atan2(centers(2,2)-centers(1,2),centers(2,1)-centers(1,1));
              bbox = [center,2*offset,centersDist+2*offset,oriention-pi/2];
              [cropImg,tform] = imgCrop(img,bbox);
              pts = detectCheckerboardPointsMine(cropImg);
              if size(pts,1)==8
                  imagePoints = transformPointsInverse(tform,pts);
              end
        end
    end

    if size(imagePoints,1)==8
        %% imagePoints排序,设计为与内建排序一致
        orderedPts = sortCheckerboardPts(imagePoints);
        plot(orderedPts(:,1),orderedPts(:,2),'ro')
        text(orderedPts(:,1)+20,orderedPts(:,2),string(1:8),Color='r')

        %% 为第一个4个小棋盘格点生成GT,假定每个棋盘格宽度350mm
        worldPoints = generateCheckerboardPoints([3,3],350);

        %% 计算2个标定模式的中心距看是否满足"2m"精度要求
        intrinsics = cameraParams.Intrinsics;
        camExtrinsics = estimateExtrinsics(orderedPts(1:4,:),worldPoints,intrinsics);
        % P = cameraProjection(intrinsics,camExtrinsics);
        worldPoints = img2world2d(orderedPts,camExtrinsics,intrinsics);
        cumDist = 0;
        for idx = 1:4
            pt1 = worldPoints(idx,:);
            pt2 = worldPoints(idx+4,:);
            cumDist = cumDist+vecnorm(pt1-pt2);
        end
        meanDist = cumDist/4;% 两个标靶圆心平均中心距
        ratio = (meanDist-2000)/2000;
        vpa(meanDist,8)
    end
end







