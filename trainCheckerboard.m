%% 初步训练一个检测棋盘格点的目标检测器
% 2023.5.29 fasterRCNN检测器至少50个epoch以上，才有效果
load data/verifyCheckerBoard.mat
[imds,blds] = objectDetectorTrainingData(gTruth);
ds = combine(imds,blds);
inputSize = [224,224,3];% derive from yolov4 official example

trainingDataForEstimation = transform(ds,@(data)preprocessData(data,inputSize));% TransformedDatastore type object

% preview data
data = read(trainingDataForEstimation);
demoImage = insertObjectAnnotation(data{1},"Rectangle",data{2},"checkerboard");
figure;imshow(demoImage);
reset(trainingDataForEstimation);
useACF = false;

%% train ACF detector
if useACF
    detector = trainACFObjectDetector(trainingDataForEstimation,NegativeSamplesFactor=2);
else
    network = 'resnet18';
    featureLayer = 'res4b_relu';
    numClasses = 1;
    numAnchors = 2;
    anchorBoxes = estimateAnchorBoxes(trainingDataForEstimation,numAnchors);
    load data/resnet18.mat
    % anchorBoxes = round(anchorBoxes.*inputSize(1:2)./size(readimage(imds,1),[1,2]));
    loadWeight = false;
    if loadWeight
        load data/fasterRcnnDetector.mat
    else
        lgraph = fasterRCNNLayers(inputSize,numClasses,anchorBoxes, ...
            resnet18,featureLayer);
    end
    options = trainingOptions('sgdm', ...
        'Shuffle','every-epoch',...
        'MiniBatchSize', 8, ...
        'InitialLearnRate', 1e-3, ...
        'MaxEpochs', 80, ...
        'VerboseFrequency', 5, ...
        'ExecutionEnvironment','gpu',...
        'CheckpointPath', "./");
    if loadWeight
        fasterRcnnDetector = trainFasterRCNNObjectDetector(trainingDataForEstimation, fasterRcnnDetector, options, ...
            'NegativeOverlapRange',[0 0.6], ...
            'PositiveOverlapRange',[0.75 1]);
    else
        fasterRcnnDetector = trainFasterRCNNObjectDetector(trainingDataForEstimation, lgraph, options, ...
            'NegativeOverlapRange',[0 0.6], ...
            'PositiveOverlapRange',[0.75 1]);
    end
end

%%
load data/fasterRcnnDetector.mat
imdsTest = imageDatastore("/opt_disk2/rd22946/AllDataAndModels/from_wangzhi/三角测距/三角标定图片",IncludeSubfolders=true,....
    FileExtensions=".bmp");
imdsTest = shuffle(imdsTest);
imdsReSz = transform(imdsTest,@(x)im2single(imresize(x,inputSize(1:2))));
for i = 1:length(imdsTest.Files)
    img = read(imdsReSz);

    if useACF
        [bboxes,scores] = detect(detector,img);
    else
        [bboxes, scores, labels] = detect(fasterRcnnDetector,img);
    end
    if ~isempty(bboxes)
        [score,idx] = maxk(scores,1);
        bbox = bboxes(idx,:);
        annotation = sprintf('Confidence = %.1f',score);
        img = insertObjectAnnotation(img,'rectangle',bbox,annotation);
    end
    figure
    imshow(img)
end


function data = preprocessData(data,targetSize)
for num = 1:size(data,1)
    I = data{num,1};
    imgSize = size(I);
    bboxes = data{num,2};
    I = im2single(imresize(I,targetSize(1:2)));
    scale = targetSize(1:2)./imgSize(1:2);
    bboxes = bboxresize(bboxes,scale);
    data(num,1:2) = {I,bboxes};
end
end

function out = preprocessImage(inImage,targetSize)
out = im2single(imresize(inImage,targetSize(1:2)));
end