% Create Training Set
% -------------------

% Created by Florian Amstutz, HS2016
% Modified by Simon Zimmermann, FS2017

%Counter for patches
patchCounter = zeros(1,10);

%Output path
if(createValidationSet)
    pathOutput = 'validationSet';
else
    pathOutput = 'trainingSet';
end

if ~exist(pathOutput,'dir')
    if(createTrainingSetForBall)
        pathOutputBall = [];
        if(cameraChoice == 1) %Upper camera
            for i = 1:4
                pathOutputBall = [pathOutputBall; strcat(pathOutput,'/',pathOutput,'BallUpper',num2str(i))];
                mkdir(pathOutputBall(i,:));
                mkdir(strcat(pathOutputBall(i,:),'/xml'));
            end
        else %lower camera
            for i = 1:2
                pathOutputBall = [pathOutputBall; strcat(pathOutput,'/',pathOutput,'BallLower',num2str(i))];
                mkdir(pathOutputBall(i,:));
                mkdir(strcat(pathOutputBall(i,:),'/xml'));
            end
        end
    else
        mkdir(pathOutput);
        mkdir(strcat(pathOutput,'/xml'));
    end
else
    error('Delete old set before you start a new one!')
end

%Cell array of xml nodes
xmlDocs = cell(1,length(classLabels));
xmlRootNodes = cell(1,length(classLabels));
patchesRootNodes = cell(1,length(classLabels));

%check whether folder for negative patches exist
pathNegative = [pathOutput,  '/', classLabels{1}, '/'];
if ~exist(pathNegative,'dir')
    mkdir(pathNegative);
end

for l = 1:numFolders %iterate over folders
    %Paths
    folder = strjoin(folders_training(l));
    pathImages = strcat(folder, '/input');
    pathAnnotations = strcat(folder, '/output/xml/annotation.xml');

    %Read xml file
    docNode = xmlread(pathAnnotations);

    %Fetch all image nodes
    imageNodes = docNode.getElementsByTagName('Image');

    numImageNodes = imageNodes.getLength;
    if numImageNodes > 0

        close all;

        %Iterate over all image nodes
        for i=1:numImageNodes
            sprintf('Folder %d of %d', l, numFolders)
            percentage = i*100/numImageNodes

            %Get image node
            currentImageNode = imageNodes.item(i-1);

            %Path to image
            imageName = char(currentImageNode.getAttribute('name'));
            pathCurrentImage = [pathImages, '/', imageName];

            [~,imageNameWithoutExtension,~] = fileparts(pathCurrentImage);
            pathCameraMatrix = [pathImages, '/xml/', imageNameWithoutExtension, '.xml'];

            %Load image
            currentImage = imread(pathCurrentImage);

            %Size of image matrix
            imageSize = size(currentImage);
            
            %Check for lower and upper camera
            if((cameraChoice == 0 && imageSize(1) == 480) || (cameraChoice == 1 && imageSize(1) == 240))
            	continue;
            end            

            %Mask of background
            backgroundMask = zeros(imageSize(1:2));

            %Get annotations of current image
            imageAnnotations = currentImageNode.getElementsByTagName('Annotation');

            numAnnotations = imageAnnotations.getLength;

            %Get camera matrix
            cameraMatrix =[];

            if exist(pathCameraMatrix,'file') == 2
                cameraMatrixDoc = xmlread(pathCameraMatrix);
                cameraMatrix = cameraMatrixDoc.getElementsByTagName('CameraMatrix').item(0);
            end

            %If annotations have been made...
            if numAnnotations > 0

                %Iterate over all annotations of current image
                for j=1:numAnnotations

                    %Get annotation
                    currentAnnotationNode = imageAnnotations.item(j-1);

                    %Class of annotation
                    currentClass = currentAnnotationNode.getAttribute('class');
                    %Class labes begin with 0
                    indexClass = str2num(currentClass) + 1;
                    indexClassConst = indexClass;

                    %Activate classMerge -> %negative / ball / robot / goal / line / robotBody / robotKnee / robotBack
                    if classMerge
                        if indexClass > 2 %Merge everything but ball
                           indexClass = 1; %negative
                        end
                    end

                    %Check whether creating training set is enabled for this class
                    if trainingSetForClass(l,indexClass)
                        
                        %Get parameters from annotation
                        x = str2num(currentAnnotationNode.getAttribute('x'));
                        y = str2num(currentAnnotationNode.getAttribute('y'));

                        %Check whether folder of class exists. If not, make new folder
                        if(createTrainingSetForBall && indexClass == 2)
                            coordLengthBallTemp = length(x);
                            ballIndexTemp = round(coordLengthBallTemp/2);
                            ballRadiusTemp = sqrt((xBallTemp(1)-xBallTemp(ballIndexTemp))^2 + (yBallTemp(1)-yBallTemp(ballIndexTemp))^2);
                            
                            if(cameraChoice == 1) %upper
                                if(ballRadiusTemp >= splitBallXUpper(1))
                                    pathBallTemp = pathOutputBall(1,:);
                                    patchSizeTemp = patchSizesBallUpper(1);
                                elseif(ballRadiusTemp < splitBallXUpper(1) && ballRadiusTemp >= splitBallXUpper(2))
                                    pathBallTemp = pathOutputBall(2,:);
                                    patchSizeTemp = patchSizesBallUpper(2);
                                elseif(ballRadiusTemp < splitBallXUpper(2) && ballRadiusTemp >= splitBallXUpper(3))
                                    pathBallTemp = pathOutputBall(3,:);
                                    patchSizeTemp = patchSizesBallUpper(3);
                                elseif(ballRadiusTemp < splitBallXUpper(3))
                                    pathBallTemp = pathOutputBall(4,:);
                                    patchSizeTemp = patchSizesBallUpper(4);
                                end

                            else %lower
                                if(ballRadiusTemp >= splitBallXLower)
                                   pathBallTemp = pathOutputBall(1,:);
                                   patchSizeTemp = patchSizesBallLower(1);
                                else
                                   pathBallTemp = pathOutputBall(2,:);
                                   patchSizeTemp = patchSizesBallLower(2);
                                end
                            end
                            pathClass = [pathBallTemp,  '/', classLabels{indexClass}, '/'];
                            
                        else
                            pathClass = [pathOutput,  '/', classLabels{indexClass}, '/'];
                            patchSizeTemp = patchSize(indexClassConst);
                        end

                        if ~exist(pathClass,'dir')
                            mkdir(pathClass);
                        end

                        %Check whether xml doc exists
                        if isempty(xmlDocs{indexClass})
                            xmlDocs{indexClass} = com.mathworks.xml.XMLUtils.createDocument('opencv_storage');
                            xmlRootNodes{indexClass} = xmlDocs{indexClass}.getDocumentElement;
                            patchesRootNodes{indexClass} = xmlDocs{indexClass}.createElement('Patches');
                        end

                        %Set annotation box in backgroundMask to 1
                        m = imageSize(1,1);
                        n = imageSize(1,2);
                        backgroundMask = poly2mask(x,y,m,n); % WITHOUT overlap!

                        %Useful parameters
                        numberOfPixels = sum(backgroundMask(:));
                        structBoundaries = bwboundaries(backgroundMask);

                        if length(structBoundaries) > 0 %check if annotation has a certain size (otherwise skip)
                            xyBoundaryCoordinates = structBoundaries{1}; % Get n by 2 array of x,y boundary coordinates
                            xBoundaryCoordinates = xyBoundaryCoordinates(:,2); % Columns
                            yBoundaryCoordinates = xyBoundaryCoordinates(:,1); % Rows

                            %Crop the image
                            topLine = min(xBoundaryCoordinates);
                            bottomLine = max(xBoundaryCoordinates);
                            leftColumn = min(yBoundaryCoordinates);
                            rightColumn = max(yBoundaryCoordinates);
                            width = bottomLine - topLine + 1;
                            height = rightColumn - leftColumn + 1;
                            croppedCurrentImage = imcrop(currentImage, [topLine, leftColumn, width, height]);
                            croppedBackgroundMask = imcrop(backgroundMask, [topLine, leftColumn, width, height]);
                            croppedBackgroundMaskSize = size(croppedBackgroundMask);

                            diameter = num2str(floor(min(width, height))); % smaller value for diameter (useful ???)

                            %Check whether cropped image size is at least patch size
                            if croppedBackgroundMaskSize(1) > patchSizeTemp && croppedBackgroundMaskSize(2) > patchSizeTemp

                                %%CREAT PATCHES INSIDE THE POLYNOMIAL
                                %Create random x and y for selection of random patch
                                randXinside = zeros(1, numPatchesInside(l,indexClassConst));
                                randYinside = zeros(1, numPatchesInside(l,indexClassConst));

                                j = 1;
                                a = 0;
                                while j <= numPatchesInside(l,indexClassConst)
                                    if a < 100000
                                        randXtemp = randi(croppedBackgroundMaskSize(2));
                                        randYtemp = randi(croppedBackgroundMaskSize(1));
                                        if croppedBackgroundMask(randYtemp,randXtemp) == 1 && randYtemp-patchSizeTemp/2 > 0 && randXtemp-patchSizeTemp/2 > 0 && randYtemp+patchSizeTemp/2-1 <= croppedBackgroundMaskSize(1) && randXtemp+patchSizeTemp/2-1 <= croppedBackgroundMaskSize(2)
                                            randXinside(1,j) = randXtemp;
                                            randYinside(1,j) = randYtemp;
                                            j = j+1;
                                        end
                                        a = a + 1;
                                    else
                                        break
                                    end
                                end

                                 for p=1:numPatchesInside(l,indexClassConst)
                                    if p < j
                                        %Creates random patches out of the annotated field (point (randX,randY) is center of patch)    
                                        currentPatch = croppedCurrentImage((randYinside(p)-patchSizeTemp/2):(randYinside(p)+patchSizeTemp/2-1), (randXinside(p)-patchSizeTemp/2):(randXinside(p)+patchSizeTemp/2-1),:);

                                        %Save new patch
                                        patchFileName = [classLabels{indexClass}, '_', num2str(patchCounter(indexClass)), '.bmp'];
                                        patchPath = [pathClass, patchFileName];

                                        imwrite(currentPatch, patchPath);

                                        %Add to xml
                                        patchNode = xmlDocs{indexClass}.createElement(['Patch', num2str(patchCounter(indexClass))]);

                                        %Name of patch
                                        nameNode = xmlDocs{indexClass}.createElement('Name');
                                        nameNode.appendChild(xmlDocs{indexClass}.createTextNode([classLabels{indexClass}, '/', patchFileName]));
                                        patchNode.appendChild(nameNode);

                                        %Diameter of cropped image (smaller value)
                                        diameterNode = xmlDocs{indexClass}.createElement('Diameter');
                                        diameterNode.appendChild(xmlDocs{indexClass}.createTextNode(diameter));
                                        patchNode.appendChild(diameterNode);

                                        %Offset to object center (= vector from center of patch to center of cropped image)
                                        offsetNode = xmlDocs{indexClass}.createElement('Offset');
                                        offset = [num2str(round(croppedBackgroundMaskSize(2)*0.5) - randXinside(p)), ' ', num2str(round(croppedBackgroundMaskSize(1)*0.5) - randYinside(p))];
                                        offsetNode.appendChild(xmlDocs{indexClass}.createTextNode(offset));
                                        patchNode.appendChild(offsetNode);

                                        %Append camera matrix if available
                                        if ~isempty(cameraMatrix)
                                            cameraMatrixClone = cameraMatrix.cloneNode(true);
                                            xmlDocs{indexClass}.adoptNode(cameraMatrixClone);
                                            patchNode.appendChild(cameraMatrixClone);
                                        end

                                        patchesRootNodes{indexClass}.appendChild(patchNode);

                                        %Increase patch counter
                                        patchCounter(indexClass) = patchCounter(indexClass) + 1;
                                    end
                                 end

                                %%CREATE PATCHES ON THE BOUNDARY OF THE POLYNOMIAL
                                %Create random x and y for selection of random patch
                                randXboundary = zeros(1, numPatchesBoundary(l,indexClassConst));
                                randYboundary = zeros(1, numPatchesBoundary(l,indexClassConst));

                                j = 1;
                                a = 0;
                                while j <= numPatchesBoundary(l,indexClassConst)
                                    if a < 100000
                                        randtemp = randi(length(xyBoundaryCoordinates));
                                        BoundaryXtemp = xBoundaryCoordinates(randtemp,1);
                                        BoundaryYtemp = yBoundaryCoordinates(randtemp,1);
                                        if BoundaryYtemp-patchSizeTemp/2 > 0 && BoundaryXtemp-patchSizeTemp/2 > 0 && BoundaryYtemp+patchSizeTemp/2-1 <= imageSize(1) && BoundaryXtemp+patchSizeTemp/2-1 <= imageSize(2)
                                            randXboundary(1,j) = BoundaryXtemp;
                                            randYboundary(1,j) = BoundaryYtemp;
                                            j = j+1;
                                        end
                                        a = a + 1;
                                    else
                                        break
                                    end
                                end

                                for p=1:numPatchesBoundary(l,indexClassConst)
                                    if p < j
                                        %Creates random patches out of the annotated field (point (randX,randY) is center of patch)    
                                        currentPatch = currentImage((randYboundary(p)-patchSizeTemp/2):(randYboundary(p)+patchSizeTemp/2-1), (randXboundary(p)-patchSizeTemp/2):(randXboundary(p)+patchSizeTemp/2-1),:);

                                        %Save new patch
                                        patchFileName = [classLabels{indexClass}, '_', num2str(patchCounter(indexClass)), '.bmp'];
                                        patchPath = [pathClass, patchFileName];

                                        imwrite(currentPatch, patchPath);

                                        %Add to xml
                                        patchNode = xmlDocs{indexClass}.createElement(['Patch', num2str(patchCounter(indexClass))]);

                                        %Name of patch
                                        nameNode = xmlDocs{indexClass}.createElement('Name');
                                        nameNode.appendChild(xmlDocs{indexClass}.createTextNode([classLabels{indexClass}, '/', patchFileName]));
                                        patchNode.appendChild(nameNode);

                                        %Diameter of cropped image (smaller value)
                                        diameterNode = xmlDocs{indexClass}.createElement('Diameter');
                                        diameterNode.appendChild(xmlDocs{indexClass}.createTextNode(diameter));
                                        patchNode.appendChild(diameterNode);

                                        %Offset to object center (= vector from center of patch to center of cropped image)
                                        offsetNode = xmlDocs{indexClass}.createElement('Offset');
                                        offset = [num2str(round(croppedBackgroundMaskSize(2)*0.5) - randXboundary(p)), ' ', num2str(round(croppedBackgroundMaskSize(1)*0.5) - randYboundary(p))];
                                        offsetNode.appendChild(xmlDocs{indexClass}.createTextNode(offset));
                                        patchNode.appendChild(offsetNode);

                                        %Append camera matrix if available
                                        if ~isempty(cameraMatrix)
                                            cameraMatrixClone = cameraMatrix.cloneNode(true);
                                            xmlDocs{indexClass}.adoptNode(cameraMatrixClone);
                                            patchNode.appendChild(cameraMatrixClone);
                                        end

                                        patchesRootNodes{indexClass}.appendChild(patchNode);

                                        %Increase patch counter
                                        patchCounter(indexClass) = patchCounter(indexClass) + 1;
                                    end
                                end
                            end
                        end
                    end
                end
            end

            if percentageWeightingForBackground(l) > 0
                %Check whether xml doc exists for negative class
                if isempty(xmlDocs{1})
                    xmlDocs{1} = com.mathworks.xml.XMLUtils.createDocument('opencv_storage');
                    xmlRootNodes{1} = xmlDocs{1}.getDocumentElement;
                    patchesRootNodes{1} = xmlDocs{1}.createElement('Patches');
                end
                
                % For ball class
                numTrainingSetsBackground = 1;
                if(createTrainingSetForBall)
                    if(cameraChoice == 1)
                        numTrainingSetsBackground = 4;
                    else
                        numTrainingSetsBackground = 2;
                    end
                end
                
                %CREATE PATCHES OUTSIDE OF THE POLYNOMIAL (= BACKGROUND PATCHES)
                for k = 1:numTrainingSetsBackground
                    
                    noBackgroundPatches = false;
                    
                    if(createTrainingSetForBall && cameraChoice == 1)
                        pathClass = [pathOutputBall(k,:),  '/', classLabels{1}, '/'];
                        patchSizeTemp = patchSizesBallUpper(k);
                    elseif(createTrainingSetForBall && cameraChoice == 0)
                        pathClass = [pathOutputBall(k,:),  '/', classLabels{1}, '/'];
                        patchSizeTemp = patchSizesBallLower(k);
                    else
                        pathClass = [pathOutput,  '/', classLabels{1}, '/'];
                        patchSizeTemp = patchSize(1);
                    end
                    
                    for j=1:numPatchesBackground(l)

                        %Search randomly a patch which does not interfere an annotation
                        isBackground = false;
                        a = 0;

                        while ~isBackground
                            %Randomly select a starting point
                            randX = randi(imageSize(2) - patchSizeTemp);
                            randY = randi(imageSize(1) - patchSizeTemp);

                            %If patch is background set isBackground to true, else repeat procedure
                            if backgroundMask(randY:(randY + patchSizeTemp - 1), randX:(randX + patchSizeTemp - 1)) == zeros(patchSizeTemp)
                                isBackground = true;
                            end

                            a = a + 1;
                            if(a > 100000)
                                noBackgroundPatches = true;
                                break;
                            end
                        end

                        if(noBackgroundPatches)
                            break;
                        end

                        %Create and save negative patch
                        currentPatch = currentImage(randY:(randY+patchSizeTemp-1), randX:(randX+patchSizeTemp-1),:);

                        patchFileName = [classLabels{1}, '_', num2str(patchCounter(1)), '.bmp'];
                        patchPath = [pathClass, patchFileName];

                        imwrite(currentPatch, patchPath);

                        %Add to xml
                        patchNode = xmlDocs{1}.createElement(['Patch', num2str(patchCounter(1))]);

                        nameNode = xmlDocs{1}.createElement('Name');
                        nameNode.appendChild(xmlDocs{1}.createTextNode([classLabels{1}, '/', patchFileName]));
                        patchNode.appendChild(nameNode);

                        diameterNode = xmlDocs{1}.createElement('Diameter');
                        diameterNode.appendChild(xmlDocs{1}.createTextNode('0'));
                        patchNode.appendChild(diameterNode);

                        %Set offset to zero
                        offsetNode = xmlDocs{1}.createElement('Offset');
                        offset = '0 0';
                        offsetNode.appendChild(xmlDocs{1}.createTextNode(offset));

                        patchNode.appendChild(offsetNode);

                        %Append camera matrix if available
                        if ~isempty(cameraMatrix)
                            cameraMatrixClone = cameraMatrix.cloneNode(true);
                            xmlDocs{1}.adoptNode(cameraMatrixClone);
                            patchNode.appendChild(cameraMatrixClone);
                        end

                        patchesRootNodes{1}.appendChild(patchNode);

                        %Increase patch counter (for old xml)
                        patchCounter(1) = patchCounter(1) + 1;

                    end
                end
            end
        end
    else
        disp('No images available.');
    end
end

%Save xml files for each class, readable for opencv FileStorage (new version)
disp('Wait for it! Saving everything...')
for i=1:length(classLabels)

    %Check if xml doc exists
    if ~isempty(xmlDocs{i})
        xmlRootNodes{i}.appendChild(patchesRootNodes{i});
        xmlwrite([pathOutput,'/xml/', classLabels{i}, '.xml'], xmlDocs{i});
    end
end

disp('End of procedure!')