function loc = find_peaks(metric,quality)
%#codegen

%   Copyright 2014-2020 The MathWorks, Inc.

if isempty(coder.target)
    % matlab
    maxMetric = max(metric(:));
    threshold = quality * maxMetric;
    loc = visionComputePeaks(metric,threshold);
else
    % codegen
    loc = vision.internal.findPeaks(metric, quality);
end
