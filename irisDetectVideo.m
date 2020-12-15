%--------------------------------------------------------------------------
% Create the face detector object.
eyeDetector = vision.CascadeObjectDetector('EyePairSmall');

% Create the webcam object.
cam = webcam();

% Capture one frame to get its size.
videoFrame = snapshot(cam);
frameSize = size(videoFrame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

runLoop = true;
frameCount = 0;
%--------------------------------------------------------------------------
while runLoop && frameCount < 600

    % Get the next frame.
    videoFrame = snapshot(cam);
    videoFrameGray = rgb2gray(videoFrame);
    frameCount = frameCount + 1;
    
    % Detection mode.
    bbox = eyeDetector.step(videoFrameGray);
    
    % Crop image after detecting eye region
    s = length(bbox(:,4));
    I2 = imcrop(videoFrameGray,bbox(s,:));
    grayImage = I2;
    %--------------------------------------------------------------------------
    % Turn to binary image to filter the skin
    newImage = ~imbinarize(grayImage,0.10);
    newImage = bwmorph(newImage,'close');
    newImage = bwmorph(newImage,'open');
    newImage = bwareaopen(newImage,200);
    bwI = imfill(newImage,'holes');
    % Morph operations done----------------------------------------------------
    % Get areas and properties
    stats = regionprops(bwI);
    N = size(stats,1);
    if N < 1 || isempty(stats)
        eyesNotDetected = []; % eyes not found
        continue
    end
    
    centroids = stats.Centroid;
    [areaMax1, iMax1] = max([stats.Area]);
    stats(iMax1).Area = 0;
    [areaMax2, iMax2] = max([stats.Area]);

     
    
    %-------------------------------------------------------------------------
    % Get correct location on original image
    stats(iMax1).BoundingBox(1) = stats(iMax1).BoundingBox(1) + bbox(1);
    stats(iMax1).BoundingBox(2) = stats(iMax1).BoundingBox(2) + bbox(2);
    stats(iMax2).BoundingBox(1) = stats(iMax2).BoundingBox(1) + bbox(1);
    stats(iMax2).BoundingBox(2) = stats(iMax2).BoundingBox(2) + bbox(2);
    
    %Get eye center
    
    center=round(stats(iMax1).Centroid);
    X=center(1) + bbox(1);
    Y=center(2) + bbox(2);
    center2=round(stats(iMax2).Centroid);
    X2=center2(1) + bbox(1);
    Y2=center2(2) + bbox(2);
    
    
    if ~isempty(bbox)
        
        
        % Convert the rectangle represented as [x, y, w, h] into an
        % M-by-2 matrix of [x,y] coordinates of the four corners. This
        % is needed to be able to transform the bounding box to display
        % the orientation of the eyes.
        bboxPoints = bbox2points(bbox(1, :));
        % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
        % format required by insertShape.
        bboxPolygon = reshape(bboxPoints', 1, []);
        % Display a bounding box around the detected eyes.
        videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);
        % Display Iris shape
        videoFrame = insertShape(videoFrame, 'Circle', [X,Y,stats(iMax1).BoundingBox(3)/2], 'LineWidth', 2);
        videoFrame = insertShape(videoFrame, 'Circle', [X2,Y2,stats(iMax2).BoundingBox(3)/2], 'LineWidth', 2);
        % Display detected centers.
        videoFrame = insertMarker(videoFrame, [X,Y], '+', 'Color', 'white');
        videoFrame = insertMarker(videoFrame, [X2,Y2], '+', 'Color', 'white');
    end

    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);

    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
end

% Clean up.
clear cam;
release(videoPlayer);
release(eyeDetector);