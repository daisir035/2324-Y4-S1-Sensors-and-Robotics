
function SBC_20_1044()
% GUI

Fig = figure('Position',[500,80,980,500]);
Pnl1 = uipanel(Fig,'Position',[0.05,0.17,0.9,0.8]);
Pnl2 = uipanel(Fig,'Position',[0.05,0.05,0.9,0.1]);
Axes1 = axes(Pnl1,'Position',[0,0,1/2,1]);
Axes2 = axes(Pnl1,'Position',[1/2,0,1/2,1]);

Bt = uicontrol(Pnl2,'style','togglebutton','String','track','Fontsize',16,...
    'Units','normalized','Position',[1/5,0,1/5,1],'Callback',@LockTarget);
drawnow

Bt2 = uicontrol(Pnl2,'style','togglebutton','String','reset','Fontsize',16,...
    'Units','normalized','Position',[4/5,0,1/5,1],'Callback',@flag1);
drawnow

Bt1 = uicontrol(Pnl2,'style','togglebutton','String','autotrack','Fontsize',16,...
    'Units','normalized','Position',[2/5,0,1/5,1],'Callback',@hsvt);
drawnow

% OPEN CAMERA
Hcamera = [];
Hobj = [];
if isempty(Hcamera)
    Hobj = videoinput('winvideo',2,'MJPG_1280x720');
    Hcamera = preview(Hobj);
    frame = getsnapshot(Hobj);  
    %frame = frame(1:gap:end,1:gap:end,:);  %Downsampling to increase speed (optional)
end

% Identify
flag = 0;
tracker = vision.PointTracker('MaxBidirectionalError',1);
gap = 1.6;  %For downsampling
objectRegion = [0,0,0,0];
objFrame = [];
points = [];
bbox = [];

% start
while 1
    if ishandle(Hcamera)
        frame = getsnapshot(Hobj);  % get frame
        if flag
                bboxPoints = bbox2points(objectRegion);
                bbox = objectRegion;
                videoFrame = frame;
                points = detectMinEigenFeatures(rgb2gray(videoFrame), 'ROI', bbox);
                tracker = vision.PointTracker('MaxBidirectionalError', 2);% 0~3
                % Initialize the tracker with the initial point locations and the initial
                points = points.Location;
                initialize(tracker, points, videoFrame);
                oldPoints = points;
                while flag
                    % get the next frame
                    %The method of identifying features is used  klt
                    %https://ch.mathworks.com/help/releases/R2019a/vision/examples/face-detection-and-tracking-using-the-klt-algorithm.html
                    frame = getsnapshot(Hobj);  
                    %frame = frame(1:gap:end,1:gap:end,:);
                    videoFrame = frame;
                    % Track the points
                    [points, isFound] = step(tracker, videoFrame);%Merge
                    visiblePoints = points(isFound, :);% Update tracking point
                    oldInliers = oldPoints(isFound, :);
    
                    if size(visiblePoints, 1) >= 2 % at least 2 points
        
                        % Estimate the geometric transformation between the old points
                        % and the new points and eliminate outliers
                        [xform, inlierIdx] = estimateGeometricTransform2D(...
                        oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);
                        oldInliers    = oldInliers(inlierIdx, :);
                        visiblePoints = visiblePoints(inlierIdx, :);
        
                        % Apply the transformation to the bounding box points
                        bboxPoints = transformPointsForward(xform, bboxPoints);
                
                        % Insert a bounding box around the object being tracked
                        bboxPolygon = reshape(bboxPoints', 1, []);
                        videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth',8);
                        videoFrame = insertText(videoFrame, [bboxPolygon(3) bboxPolygon(4)-40], 'Robot', 'FontSize', 50,'TextColor','yellow', 'BoxOpacity',0);
        
                        % Draw direction arrow
                        %arrow_end = [0.5*(bboxPolygon(1)+bboxPolygon(3)), 0.5*(bboxPolygon(2)+bboxPolygon(4))];
                        %arrow_start = [0.5*(bboxPolygon(5)+bboxPolygon(7)), 0.5*(bboxPolygon(6)+bboxPolygon(8))];
                        %arrow_start1 = [0.5*(0.5*(bboxPolygon(1)+bboxPolygon(7))+arrow_end(1)), 0.5*(0.5*(bboxPolygon(2)+bboxPolygon(8))+arrow_end(2))];
                        %arrow_start2 = [0.5*(0.5*(bboxPolygon(3)+bboxPolygon(5))+arrow_end(1)), 0.5*(0.5*(bboxPolygon(4)+bboxPolygon(6))+arrow_end(2))];
                        %videoFrame = insertShape(videoFrame, 'Line', [arrow_start arrow_end;arrow_start1 arrow_end;arrow_start2 arrow_end], 'LineWidth', 3, 'Color', 'red');

                        % Reset the points
                        oldPoints = visiblePoints;
                        setPoints(tracker, oldPoints);        
                    end  
                         
                %writeVideo(outputVideo, videoFrame);
                imshow(videoFrame,'Parent',Axes1)
                %frame = getsnapshot(Hobj);  % get frame
              end
        else
        %normal display
        I = rgb2hsv(frame);
        channel1Min = 0.000;
        channel1Max = 1.000;

        % Define thresholds for channel 2 based on histogram settings
        channel2Min = 0.000;
        channel2Max = 1.000;

        % Define thresholds for channel 3 based on histogram settings
        channel3Min = 0.000;
        channel3Max = 0.549;
        % Create mask based on chosen histogram thresholds
        binaryFrame = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
        (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
        (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);

        threshold_area = 20;
        cleanedFrame = bwareaopen(binaryFrame, threshold_area); 

        stats = regionprops(cleanedFrame, 'BoundingBox'); 
        for k = 1 : length(stats)
            boundingBox = stats(k).BoundingBox;
            if (boundingBox(3)* boundingBox(4))<50000 &&(boundingBox(3)* boundingBox(4))>2500
                frame = insertShape(frame, 'Rectangle', boundingBox, 'LineWidth', 5, 'Color', 'r');
                frame = insertText(frame, [boundingBox(1) boundingBox(2)-40], 'Robot', 'FontSize', 50,'TextColor','yellow', 'BoxOpacity',0);
            end
        end

        imshow(frame,'Parent',Axes1)
    end
        drawnow
    else
        break
    end
end

function bbox = LockTarget(~,~)
        %frame = frame(1:gap:end,1:gap:end,:);
        objFrame = frame;
        % Target location
        [x,y,~] = ginput(2);
        objectRegion = [min(x),min(y),max(x)-min(x),max(y)-min(y)];
        bbox = [min(x),min(y),max(x)-min(x),max(y)-min(y)];
        % Detection feature point
        points = detectMinEigenFeatures(rgb2gray(frame),'ROI',objectRegion);%Detect corners using minimum eigenvalue algorithm and return cornerPoints object
        pointImage = insertMarker(frame, points.Location,'+','Color','green');
        imshow(pointImage,'Parent',Axes2)
        flag = 1;
end

function flag1(~,~)
    flag = 0;
end

function  hsvt(~,~)
    I = rgb2hsv(frame);
    % Define thresholds for channel 1 based on histogram settings
    channel1Min = 0.000;
    channel1Max = 1.000;

    % Define thresholds for channel 2 based on histogram settings
    channel2Min = 0.000;
    channel2Max = 1.000;

    % Define thresholds for channel 3 based on histogram settings
    channel3Min = 0.000;
    channel3Max = 0.549;
    % Create mask based on chosen histogram thresholds
    binaryFrame = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
        (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
        (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);

    threshold_area = 20;
    cleanedFrame = bwareaopen(binaryFrame, threshold_area); 

    stats = regionprops(cleanedFrame, 'BoundingBox'); 
    for k = 1 : length(stats)
        boundingBox = stats(k).BoundingBox;
        if (boundingBox(3)* boundingBox(4))<50000&&(boundingBox(3)* boundingBox(4))>2500 %Selective area size
            objectRegion=boundingBox;
            points = detectMinEigenFeatures(rgb2gray(frame),'ROI',boundingBox);
            pointImage = insertMarker(frame, points.Location,'+','Color','green');
            imshow(pointImage,'Parent',Axes2)
            break;
        end
    end
    flag =1;
end
end
