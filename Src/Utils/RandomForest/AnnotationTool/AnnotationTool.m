% Image Annotation Tool                 
% -----------------------
% Created by Florian Amstutz, HS2016 
% Modified by Simon Zimmermann, FS2017
% 
% The purpose of this tool is to create patches of annotated objects on images
% for a random forest object detection.
%
% Instructions:                           
% -------------   
% The script consists of three sections:
%
% - In the first section global settings are defined which can be modified.
% - The second section starts the annotation process. 
% - The third section creates the patches out of the annotated boxes and saves xml
%   files that are used for the random forest tree building process.
%
% Keys for annotation: 
% 
% ESC:        Get out of drawing mode
% 1,2,3,..,0: Assign annotation to corresponding class. Drawing tool will
%             again be activated. Need to be closed with ESC to continue
%             with another command. 0 is assigned to the negative class/background.
% SPACE:      Go to next image, save current annotations. If only a single
%             drawing box exists, it will be assigned to the default class 
% BACKSPACE:  Delete current annotations (drawing tool has to be turn off
%             via ESC)

%% 0) Tabula Rasa
clc
close all
clear all


%% 1) Set Parameters for Annotation
%Folder name
folder_annotation = 'acq_lower_ETH_20171012';

%Zoom for annotation window
%imageZoom = 100; %Standard zoom
% imageZoom = 170; %External display 
imageZoom = 300; %High resolution display

%If true, images are converted to RGB for annotation (Does not change color
%space of patches!)
convertFromYCrCb = true;

%Class labels and colors for boxes. Negative class has number/index 0 (index begins with 0!!)
%negative = 0 = black / ball = 1 = magenta / robot = 2 = cyan / goal = 3 = yellow / line = 4 = red
classColors = 'kmcyrbwwww';
classLabels = [{'negative'},{'ball'},{'robot'},{'goal'},{'line'},{'robotBody'},{'robotKnee'},{'robotBack'},{'empty'},{'empty'}];

%Default class
defaultClass = 4;

%Start where left from last annotation
startWhereLeft = false;

%Default drawing style (0 = polygon / 1 = ellipse / 2 = rectangle)
defaultDrawingStyle = 0;

%% 2) Start Annotation
createAnnotations;


%% 3) Set Parameters for Patch Creation
%!!! Make sure that "classColors" & "classLabels" are from point 1) are in
%the workspace !!!

%Splits for ball class
splitBallRegionUpper = [35, 21, 12]; %Ball radius
splitBallYUpper = [0.75*480, 0.5*480, 0.25*480];
patchSizesBallUpper = [38, 28, 16, 10];
splitBallRegionLower = 30;
patchSizesBallLower = [32, 26];
splitBallYLower = 0.5*240;

%If true, create validation set, otherwise create training set
createValidationSet = false;

%Folders which you want to use for creation of training set
folders_training = {'acquisition_ETH_17_06_24', ...
                    'acquisition_ETH_17_06_24_decreased', ...
                    'acquisition_ETH_17_06_24_increased', ...
                    'acquisition_ETH_17_06_27', ...
                    'acquisition_ETH_17_03_30_part', ...
                    'acquisition_Magdeburg_17_05_03_part', ...
                    'acquisition_nagoya_170725_training1', ...
                    'acquisition_nagoya_170725_training2', ...
                    'acquisition_nagoya_170725_training3', ...
                    'acq_lower_ETH_20171012'};

%Creates only patches for classes which have a one at corresponding index
%(row = folder / col = classes)
trainingSetForClass = [1,1,1,1,1,0,0,0,0,0; %ETH
                       1,1,1,1,1,0,0,0,0,0; %ETH
                       1,1,1,1,1,0,0,0,0,0; %ETH
                       1,1,1,1,1,0,0,0,0,0; %ETH
                       1,1,1,1,1,0,0,0,0,0; %ETH
                       1,1,1,1,1,0,0,0,0,0; %Magdeburg
                       1,1,1,1,1,1,1,1,0,0; %Nagoya
                       1,1,1,1,1,1,1,1,0,0; %Nagoya
                       1,1,1,1,1,1,1,1,0,0; %Nagoya
                       1,1,1,1,1,1,0,0,0,0];%ETH

%Weighting per folder (how many patches you want to get out of which
%folder)
percentageWeightingPerFolder = [0.10, 0.05, 0.05, 0.10, 0.05, 0.05, 0.05, 0.05, 0.05, 0.45];
percentageWeightingForBackground = [0.10, 0.05, 0.05, 0.10, 0.05, 0.05, 0.05, 0.05, 0.05, 0.45];

%Target number of patches per class
nP = 100;
targetNumPatches = [0.9*nP; 0.1*nP; 0.8*nP; 0.1*nP; nP; nP; 0.05*nP; 0.05*nP; 0; 0]; %!!!!
%negative / ball / robot / goal / line / robotBody / robotKnee / robotBack

%Percentage of patches that should be taken from the boundary of the
%annotation (rest is taken from the inside)
percentageForBoundary = [0; 0.3; 0.1; 0.2; 0.3; 0; 0; 0; 0; 0];

%Init backupPercentage
initBackupPercentage = [1; 1; 0; 1; 1; 1; 1; 1; 0; 0];

%Percentage of patches that should be taken from annotated background (rest
%is taken from unnotated area of image)
percentagePatchesAnnotatedBackground = 0;

%True if one wants to merge classes ("createTrainingSet" has to be
%addapted! -> around line 97)
classMerge = true;

%Load number of annotations per folder if true, otherwise count them
loadNumAnnotations = false;

%-----------------------Nothing to change down here------------------------
%Get number of annotations per folder per class
numFolders = length(folders_training);
numAnnotationsPerClass = zeros(numFolders,10);
numImagesPerFolder = zeros(numFolders,1);
numAnnotationsPerRegionUpper = zeros(4, 10);
numAnnotationsPerRegionLower = zeros(2, 10);

if(loadNumAnnotations) 
    %loads numAnnotationsPerClass
    load('numAnnotationsPerClass_9.mat');
    numAnnotationsPerClass

    %calculates numImagesPerFolder
    for l = 1:numFolders
        %Annotation file
        folderName = strjoin(folders_training(l));
        pathAnnotations = strcat(folderName, '/output/xml/annotation.xml');

        %Get number of images
        imageList = dir(strcat(folderName, '/input'));
        imageList = imageList(arrayfun(@(x) ~strcmp(x.name(1),'.'),imageList));
        imageList = {imageList(~[imageList.isdir]).name};
        numImages = length(imageList);
        numImagesPerFolder(l) = numImages;
    end

    numImagesPerFolder

else
    %Iterate over all images
    for l = 1:numFolders
        %Images
        folder = char(folders_training(l));
        pathImages = strcat(folder, '/input');

        %Annotation file
        folderName = strjoin(folders_training(l));
        pathAnnotations = strcat(folderName, '/output/xml/annotation.xml');

        %Get number of images
        imageList = dir(strcat(folderName, '/input'));
        imageList = imageList(arrayfun(@(x) ~strcmp(x.name(1),'.'),imageList));
        imageList = {imageList(~[imageList.isdir]).name};
        numImages = length(imageList);
        numImagesPerFolder(l) = numImages;

        %Read xml file
        annotationsNode = xmlread(pathAnnotations);
        annotationsRootNode = annotationsNode.getDocumentElement;

        imageNodes = annotationsRootNode.getElementsByTagName('Image');
        numImageNodes = imageNodes.getLength;

        for i=1:numImages
            %Initialize nodeExists
            nodeExists = false;

            %Get image node
            currentImageNode = imageNodes.item(i-1);

            %Path to image
            imageName = char(currentImageNode.getAttribute('name'));

            pathCurrentImage = strcat(pathImages, '/', imageName);

            %Load image
            currentImage = imread(pathCurrentImage);

            %Size of image matrix
            imageSize = size(currentImage);

            %Check for lower and upper camera
            if(imageSize(1) == 480)
                cameraChoice = 1; %Upper camera
            elseif(imageSize(1) == 240)
                cameraChoice = 0; %Lower camera
            else
                error('Image size?!')
            end

            %Load annotations
            for j=1:numImageNodes

                if imageNodes.item(j-1).getAttribute('name') == imageList{i}
                    nodeExists = true;
                    annotationNodes = imageNodes.item(j-1).getElementsByTagName('Annotation');

                    numberAnnotations = annotationNodes.getLength;

                    if numberAnnotations > 0
                        for k=1:numberAnnotations

                            annotationNode = annotationNodes.item(k-1);
                            key = annotationNode.getAttribute('class');
                            indexClass = str2double(key) + 1;

                            numAnnotationsPerClass(l,indexClass) = numAnnotationsPerClass(l,indexClass) + 1;

                            xTemp = str2num(annotationNode.getAttribute('x'));
                            yTemp = str2num(annotationNode.getAttribute('y'));

                            %Check ball radius
                            if(indexClass == 2) %Ball class
                                coordLengthBallTemp = length(xTemp);
                                ballIndexTemp = round(coordLengthBallTemp/2);
                                ballRadiusTemp = (sqrt((xTemp(1)-xTemp(ballIndexTemp))^2 + (yTemp(1)-yTemp(ballIndexTemp))^2))/2;

                                if(cameraChoice == 1) %upper
                                    if(ballRadiusTemp >= splitBallRegionUpper(1))
                                        numAnnotationsPerRegionUpper(1, 2) = numAnnotationsPerRegionUpper(1, 2) + 1;
                                    elseif(ballRadiusTemp < splitBallRegionUpper(1) && ballRadiusTemp >= splitBallRegionUpper(2))
                                        numAnnotationsPerRegionUpper(2, 2) = numAnnotationsPerRegionUpper(2, 2) + 1; 
                                    elseif(ballRadiusTemp < splitBallRegionUpper(2) && ballRadiusTemp >= splitBallRegionUpper(3))
                                        numAnnotationsPerRegionUpper(3, 2) = numAnnotationsPerRegionUpper(3, 4) + 1;
                                    elseif(ballRadiusTemp < splitBallRegionUpper(3))
                                        numAnnotationsPerRegionUpper(4, 2) = numAnnotationsPerRegionUpper(4, 2) + 1; 
                                    end

                                else %lower
                                    if(ballRadiusTemp >= splitBallRegionLower)
                                        numAnnotationsPerRegionLower(1, 2) = numAnnotationsPerRegionLower(1, 2) + 1;
                                    else
                                        numAnnotationsPerRegionLower(2, 2) = numAnnotationsPerRegionLower(2, 2) + 1;
                                    end
                                end
                            else %For all remaining classes
                                polygonOld = [xTemp', yTemp'];

                                if(cameraChoice == 1)
                                    m = 480;
                                    n = 640;

                                    %FIRST REGION
                                    line = [0, splitBallYUpper(1); n, splitBallYUpper(1)];
                                    side = 1; %below line

                                    polygonNew = cutpolygon(polygonOld, line, side);
                                    backgroundMask = poly2mask(polygonNew(:,1),polygonNew(:,2),m,n);
                                    numberOfPixels = sum(backgroundMask(:));
                                    if(numberOfPixels > patchSizesBallUpper(1)^2)
                                        numAnnotationsPerRegionUpper(1,indexClass) = numAnnotationsPerRegionUpper(1,indexClass) + 1;
                                    end

                                    %SECOND REGION (two cuts)
                                    side = 2; %above line
                                    polygonNew = cutpolygon(polygonOld, line, side);

                                    if(size(polygonNew,1) >= 2 && size(polygonNew,2) >= 2)
                                        line = [0, splitBallYUpper(2); n, splitBallYUpper(2)];
                                        side = 1; %below line
                                        polygonNew = cutpolygon(polygonNew, line, side);
                                    end

                                    backgroundMask = poly2mask(polygonNew(:,1),polygonNew(:,2),m,n);
                                    numberOfPixels = sum(backgroundMask(:));
                                    if(numberOfPixels > patchSizesBallUpper(2)^2)
                                        numAnnotationsPerRegionUpper(2,indexClass) = numAnnotationsPerRegionUpper(2,indexClass) + 1;
                                    end

                                    %THIRD REGION (two cuts)
                                    line = [0, splitBallYUpper(2); n, splitBallYUpper(2)];
                                    side = 2; %above line
                                    polygonNew = cutpolygon(polygonOld, line, side);

                                    if(size(polygonNew,1) >= 2 && size(polygonNew,2) >= 2)
                                        line = [0, splitBallYUpper(3); n, splitBallYUpper(3)];
                                        side = 1; %below line
                                        polygonNew = cutpolygon(polygonNew, line, side);
                                    end

                                    backgroundMask = poly2mask(polygonNew(:,1),polygonNew(:,2),m,n);
                                    numberOfPixels = sum(backgroundMask(:));
                                    if(numberOfPixels > patchSizesBallUpper(3)^2)
                                        numAnnotationsPerRegionUpper(3,indexClass) = numAnnotationsPerRegionUpper(3,indexClass) + 1;
                                    end

                                    %FOURTH REGION
                                    side = 2; %above line
                                    polygonNew = cutpolygon(polygonOld, line, side);

                                    backgroundMask = poly2mask(polygonNew(:,1),polygonNew(:,2),m,n);
                                    numberOfPixels = sum(backgroundMask(:));
                                    if(numberOfPixels > patchSizesBallUpper(4)^2)
                                        numAnnotationsPerRegionUpper(4,indexClass) = numAnnotationsPerRegionUpper(4,indexClass) + 1;
                                    end

                                else %Lower Camera
                                    m = 240;
                                    n = 320;

                                    %FIRST REGION
                                    line = [0, splitBallYLower; n, splitBallYLower];
                                    side = 1; %below line
                                    polygonNew = cutpolygon(polygonOld, line, side);
                                    backgroundMask = poly2mask(polygonNew(:,1),polygonNew(:,2),m,n);
                                    numberOfPixels = sum(backgroundMask(:));
                                    if(numberOfPixels > patchSizesBallLower(1)^2)
                                        numAnnotationsPerRegionLower(1,indexClass) = numAnnotationsPerRegionLower(1,indexClass) + 1;
                                    end

                                    %SECOND REGION
                                    side = 2; %above line
                                    polygonNew = cutpolygon(polygonOld, line, side);
                                    backgroundMask = poly2mask(polygonNew(:,1),polygonNew(:,2),m,n);
                                    numberOfPixels = sum(backgroundMask(:));
                                    if(numberOfPixels > patchSizesBallLower(2)^2)
                                        numAnnotationsPerRegionLower(2,indexClass) = numAnnotationsPerRegionLower(2,indexClass) + 1;
                                    end
                                end
                            end
                        end
                    end
                end
            end
            if(mod(i,10) == 0)
                sprintf('Folder %d of %d', l, numFolders)
                percentage = i*100/numImages
            end
        end
    end
end

for z = 1:6
    
    if(z <= 4)
    	cameraChoice = 1; %Upper camera
        numBall = z;
        numAnnotationsPerRegion = numAnnotationsPerRegionUpper;
    else
        cameraChoice = 0; %Lower camera
        numBall = z - 4;
        numAnnotationsPerRegion = numAnnotationsPerRegionLower;
    end
    
    %Init backupPercentage
    backupPercentage = initBackupPercentage;
    
    %% 4) Create Set Of Patches For Training
    enoughPatchesGenerated = false;
    while(~enoughPatchesGenerated)
        numAnnotationsPerClass = numAnnotationsPerRegion(k, :);

        numAnnotationsPerClass = numAnnotationsPerClass.*trainingSetForClass;
        if numFolders == 1
            sumNumAnnotationsPerClass = numAnnotationsPerClass;
        else
            sumNumAnnotationsPerClass = sum(numAnnotationsPerClass)';
        end
        sumNumImagesPerFolder = sum(numImagesPerFolder);

        numPatchesInside = zeros(numFolders,10);
        numPatchesBoundary = zeros(numFolders,10);
        numPatchesBackground = zeros(numFolders,1);

        for l = 1:numFolders %iterate over folders
            for i = 1:10 %iterate over classes
               if(sumNumAnnotationsPerClass(i) > 0)
                   numPatchesPerAnnotation = ceil(trainingSetForClass(l,i)*targetNumPatches(i)/sumNumAnnotationsPerClass(i)*percentageWeightingPerFolder(l));
               else
                   numPatchesPerAnnotation = 0;
               end

               numPatchesPerAnnotationPerFolderInside = ceil((1-percentageForBoundary(i))*numPatchesPerAnnotation);
               numPatchesPerAnnotationPerFolderBoundary = ceil(percentageForBoundary(i)*numPatchesPerAnnotation);

               if(i == 1) %background
                   numPatchesInside(l,i) = ceil(percentagePatchesAnnotatedBackground*(1+backupPercentage(i))*numPatchesPerAnnotationPerFolderInside);
                   numPatchesBoundary(l,i) = ceil(percentagePatchesAnnotatedBackground*(1+backupPercentage(i))*numPatchesPerAnnotationPerFolderBoundary);

               else %other classes
                   numPatchesInside(l,i) = ceil((1+backupPercentage(i))*numPatchesPerAnnotationPerFolderInside);
                   numPatchesBoundary(l,i) = ceil((1+backupPercentage(i))*numPatchesPerAnnotationPerFolderBoundary);
               end
            end
        end

        tempSumPatchesBackground = 0;
        for l = 1:numFolders
            tempSumPatchesBackground = tempSumPatchesBackground + (numPatchesInside(l,1)+numPatchesBoundary(l,1))*numImagesPerFolder(l); 
        end

        for l = 1:numFolders
            numPatchesBackground(l) = ceil((1+backupPercentage(1))*(targetNumPatches(1) - tempSumPatchesBackground)*percentageWeightingForBackground(l)/numImagesPerFolder(l));
        end
        
        [enoughPatchesGenerated, backupPercentage] = createTrainingSetBallFct(createValidationSet, ...
                                                        cameraChoice, ...
                                                        numBall, ...
                                                        classLabels, ...
                                                        folders_training, ...
                                                        classMerge, ...
                                                        trainingSetForClass, ...
                                                        numPatchesInside, ...
                                                        numPatchesBoundary, ...
                                                        numPatchesBackground, ...
                                                        patchSizesBallUpper, ...
                                                        patchSizesBallLower, ...
                                                        percentageWeightingForBackground, ...
                                                        splitBallRegionUpper, ...
                                                        splitBallRegionLower, ...
                                                        splitBallYUpper, ...
                                                        splitBallYLower, ...
                                                        nP, backupPercentage);
    end
end
