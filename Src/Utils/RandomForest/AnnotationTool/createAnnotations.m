% Create Annotations
% ------------------

% Created by Florian Amstutz, HS2016
% Modified by Simon Zimmermann, HS2017

%Maximum number of points that can be drawn for annotation polygon
maxNumPoints = 100;
ellipsdisc = pi/16; %ellipse discretization

%Paths
annotationFile = 'annotation.xml';
pathImages = strcat(folder_annotation,'/','input'); %Path to images
pathOutput = strcat(folder_annotation,'/','output'); %Path to xml files
pathAnnotations = strcat(pathOutput, '/xml/', annotationFile); %Path to annotation files

%Make sure folder xml exists
if ~exist(pathAnnotations,'file')
    %Create path
    mkdir([pathOutput,'/xml']);
    
    %Creates new xml file
    annotationsNode = com.mathworks.xml.XMLUtils.createDocument('Annotations');
    annotationsRootNode = annotationsNode.getDocumentElement;
    annotationsRootNode.setAttribute('nextImage','1');
    numImageNodes = 0;
    xmlwrite(pathAnnotations,annotationsNode);
    
    nextImage = 1;
   
else
    %Read xml file
    annotationsNode = xmlread(pathAnnotations);
    annotationsRootNode = annotationsNode.getDocumentElement;
    
    imageNodes = annotationsRootNode.getElementsByTagName('Image');
    numImageNodes = imageNodes.getLength;
    
    if startWhereLeft
        nextImage = str2num(annotationsRootNode.getAttribute('nextImage'));
    else
        nextImage = 1;
    end
end

%Load all files from directory, filter out hidden files (start with a dot) and folders
imageList = dir(pathImages);
imageList = imageList(arrayfun(@(x) ~strcmp(x.name(1),'.'),imageList));
imageList = {imageList(~[imageList.isdir]).name};
sort(imageList);

%Number of images
numImages = length(imageList);

%Check if iterated over all data
if nextImage > numImages
    error('All images annotated.');
    exit
end

%Create Figure
imageFigure = figure;

%Turn off menubar
set(imageFigure, 'menubar', 'none','NumberTitle','off');

%Iterate over all images:
for i=nextImage:numImages
    
    %Show ith image
    pathImage = [pathImages,'/', imageList{i}];
    
    %Set new title
    set(imageFigure,'name', ['AnnotationTool ','image ', num2str(i), ' of ', num2str(numImages)]);
    nextImage = false;
    
    %Load image
    imageMat = imread(pathImage);
    imageSize = size(imageMat);

    %Check if image has to be converted to RGB
    if convertFromYCrCb
        tempImage = imageMat;
        tempImage(:,:,2) = imageMat(:,:,3);
        tempImage(:,:,3) = imageMat(:,:,2);
        imageMat = ycbcr2rgb(tempImage);
    end
    
    %Matrix for saving annotations of current image
    Annotations = [];
    nodeExists = false;
    
    %Check whether image was already annotated --> load annotations
    for j=1:numImageNodes
        
        if imageNodes.item(j-1).getAttribute('name') == imageList{i}
            nodeExists = true;
            annotationNodes = imageNodes.item(j-1).getElementsByTagName('Annotation');
            
            numberAnnotations = annotationNodes.getLength;
            
            if numberAnnotations > 0
                
                imshow(imageMat,'InitialMagnification', imageZoom);
                
                text(-80,-15,['ESC = change mode /'],'Color','black','FontSize',12)
                text(83,-15,['p = draw polygon /'],'Color','black','FontSize',12)
                text(220,-15,['e = draw ellipse /'],'Color','black','FontSize',12)
                text(343,-15,['r = draw rectangle /'],'Color','black','FontSize',12)
                text(482,-15,['backspace = delete /'],'Color','black','FontSize',12)
                text(632,-15,['space = next image'],'Color','black','FontSize',12)
                
                text(-80,imageSize(1)+20,['Classes:'],'Color',classColors(1),'FontSize',12)
                text(0*imageSize(2)/10,imageSize(1)+20,['0 = ',classLabels(1)],'Color',classColors(1),'FontSize',12)
                text(1*imageSize(2)/10,imageSize(1)+20,['1 = ',classLabels(2)],'Color',classColors(2),'FontSize',12)
                text(2*imageSize(2)/10,imageSize(1)+20,['2 = ',classLabels(3)],'Color',classColors(3),'FontSize',12)
                text(3*imageSize(2)/10,imageSize(1)+20,['3 = ',classLabels(4)],'Color',classColors(4),'FontSize',12)
                text(4*imageSize(2)/10,imageSize(1)+20,['4 = ',classLabels(5)],'Color',classColors(5),'FontSize',12)
                text(5*imageSize(2)/10,imageSize(1)+20,['5 = ',classLabels(6)],'Color',classColors(6),'FontSize',12)
                text(6*imageSize(2)/10,imageSize(1)+20,['6 = ',classLabels(7)],'Color',classColors(7),'FontSize',12)
                text(7*imageSize(2)/10,imageSize(1)+20,['7 = ',classLabels(8)],'Color',classColors(8),'FontSize',12)
                text(8*imageSize(2)/10,imageSize(1)+20,['8 = ',classLabels(9)],'Color',classColors(9),'FontSize',12)
                text(9*imageSize(2)/10,imageSize(1)+20,['9 = ',classLabels(10)],'Color',classColors(10),'FontSize',12)
                
                for k=1:numberAnnotations
                    
                    annotationNode = annotationNodes.item(k-1);
                    key = annotationNode.getAttribute('class');
                    pointNum = str2num(annotationNode.getAttribute('pointNum'));
                    x = str2num(annotationNode.getAttribute('x'));
                    y = str2num(annotationNode.getAttribute('y'));
                    
%                     polygon = [x', y'];
%                     line = [0, 120; 640, 120];
%                     side = 2;
%                     Pc = cutpolygon(polygon, line, side)
%                     x = Pc(:,1)'
%                     y = Pc(:,2)'
%                     pointNum = size(x,2)
                    
                    %Save positions in Annotations matrix
                    z = zeros(1,2*maxNumPoints+2);
                    Annotations = [Annotations; z];
                    Annotations(end,1) = str2double(key);
                    Annotations(end,2) = pointNum;
                    Annotations(end,3:pointNum+2) = x;
                    Annotations(end,pointNum+3:2*pointNum+2) = y;
                    
                    %Get position parameter for patch drawing
                    f = zeros(1,pointNum);
                    position = zeros(pointNum,2);
                    for k = 1:pointNum
                        f(1,k) = k;
                    end
                    position(:,1) = x';
                    position(:,2) = y';
                    patch('Faces',f,'Vertices',position,'EdgeColor',classColors(str2double(key)+1),'FaceColor','none','LineWidth',3);
                 end
            end
            break
        end
    end
    
    %Repeat until right space has been pressed
    while ~nextImage
        
        %Show image
        if isempty(Annotations)
            imshow(imageMat,'InitialMagnification', imageZoom);
            
            text(-80,-15,['ESC = change mode /'],'Color','black','FontSize',12)
            text(83,-15,['p = draw polygon /'],'Color','black','FontSize',12)
            text(220,-15,['e = draw ellipse /'],'Color','black','FontSize',12)
            text(343,-15,['r = draw rectangle /'],'Color','black','FontSize',12)
            text(482,-15,['backspace = delete /'],'Color','black','FontSize',12)
            text(632,-15,['space = next image'],'Color','black','FontSize',12)

            text(-80,imageSize(1)+20,['Classes:'],'Color',classColors(1),'FontSize',12)
            text(0*imageSize(2)/10,imageSize(1)+20,['0 = ',classLabels(1)],'Color',classColors(1),'FontSize',12)
            text(1*imageSize(2)/10,imageSize(1)+20,['1 = ',classLabels(2)],'Color',classColors(2),'FontSize',12)
            text(2*imageSize(2)/10,imageSize(1)+20,['2 = ',classLabels(3)],'Color',classColors(3),'FontSize',12)
            text(3*imageSize(2)/10,imageSize(1)+20,['3 = ',classLabels(4)],'Color',classColors(4),'FontSize',12)
            text(4*imageSize(2)/10,imageSize(1)+20,['4 = ',classLabels(5)],'Color',classColors(5),'FontSize',12)
            text(5*imageSize(2)/10,imageSize(1)+20,['5 = ',classLabels(6)],'Color',classColors(6),'FontSize',12)
            text(6*imageSize(2)/10,imageSize(1)+20,['6 = ',classLabels(7)],'Color',classColors(7),'FontSize',12)
            text(7*imageSize(2)/10,imageSize(1)+20,['7 = ',classLabels(8)],'Color',classColors(8),'FontSize',12)
            text(8*imageSize(2)/10,imageSize(1)+20,['8 = ',classLabels(9)],'Color',classColors(9),'FontSize',12)
            text(9*imageSize(2)/10,imageSize(1)+20,['9 = ',classLabels(10)],'Color',classColors(10),'FontSize',12)
                
        end
        
        %New patch can be drawn
        if(defaultDrawingStyle == 0)
            h = impoly;
            drawingStyle = 0;
        elseif(defaultDrawingStyle == 1)
            h = imellipse;
            drawingStyle = 1;
        elseif(defaultDrawingStyle == 2)
            h = imrect;
            drawingStyle = 2;
        else
            error('defaultDrawingStyle not valid')
        end
        
        %Check if figure still exists
        if ~ishandle(imageFigure)
            error('Figure closed.');
        end
        
        validKey = false;
        
        %Repeat until a valid key has been pressed
        while ~validKey
                       
            %Wait for a key to be pressed (returns 0 for mouseclick, 1 for buttonclick)
            while ~waitforbuttonpress
            end
            
            %Get key
            key = get(gcf,'CurrentKey');
            
            %If a digit has been pressed, annotate bounding box for corresponding class
            if isstrprop(key,'digit')
                %Gets position of patch
                position = getPosition(h);
                
                if(drawingStyle == 0)
                    %Polygon is drawn -> no manipulation has to be done
                    
                elseif(drawingStyle == 1) %Ellipse is drawn
                    xMid = position(1) + position(3)/2;
                    yMid = position(2) + position(4)/2;
                    
                    a = position(3)/2;
                    b = position(4)/2;

                    position = [];
                    
                    for ellipsAngle = 0:ellipsdisc:2*pi
                        xTemp = xMid + a*cos(ellipsAngle);
                        yTemp = yMid + b*sin(ellipsAngle);
                        
                        position = [position; xTemp yTemp];
                    end
                    
                elseif(drawingStyle == 2) %Rectangle is drawn
                    positionTemp = position;
                    position = zeros(4,2);
                    position(1,1) = positionTemp(1);
                    position(1,2) = positionTemp(2);
                    position(2,1) = positionTemp(1);
                    position(2,2) = positionTemp(2) + positionTemp(4);
                    position(3,1) = positionTemp(1) + positionTemp(3);
                    position(3,2) = positionTemp(2) + positionTemp(4);
                    position(4,1) = positionTemp(1) + positionTemp(3);
                    position(4,2) = positionTemp(2);
                    
                else
                    error('drawingStyle not valid')
                end
                
                if length(position) <= maxNumPoints
                    f = zeros(1,length(position));
                    z = zeros(1,2*maxNumPoints+2);
                    Annotations = [Annotations; z];
                    Annotations(end,1) = str2double(key);
                    Annotations(end,2) = length(position);
                                            
                    for k = 1:length(position)
                        f(1,k) = k;

                        %Save positions in Annotations matrix
                        Annotations(end, k+2) = position(k,1);   %x values
                        Annotations(end, k+2+length(position)) = position(k,2); %y values
                    end
                    
                    %Delete dynamic bounding box and create a solid one with corresponding class color
                    delete(h);
                    patch('Faces',f,'Vertices',position,'EdgeColor',classColors(str2double(key)+1),'FaceColor','none','LineWidth',3);
                    
                else
                    disp('ERROR: Too many points -> Increase variable "maxNumPoints"')
                    delete(h)
                end
                
                %Set h to zero and set validKey to true
                h = 0;
                validKey = true;
            
            %Drawing mode of polygon
            elseif strcmp(key,'p')
                h = impoly;
                drawingStyle = 0;
                
            %Drawing mode of ellipse
            elseif strcmp(key, 'e')
                h = imellipse;
                drawingStyle = 1;
                
            %Drawing mode of rectangle
            elseif strcmp(key,'r')
                h = imrect;
                drawingStyle = 2;
                
            %Backspace will delete last annotation of current picture
            elseif strcmp(key,'backspace')
                
                %Save annotations temporary and delete all of them
                tempAnnotations = Annotations;
                Annotations = [];
                
                %Reload image
                imshow(imageMat,'InitialMagnification', imageZoom)
                
                text(-80,-15,['ESC = change mode /'],'Color','black','FontSize',12)
                text(83,-15,['p = draw polygon /'],'Color','black','FontSize',12)
                text(220,-15,['e = draw ellipse /'],'Color','black','FontSize',12)
                text(343,-15,['r = draw rectangle /'],'Color','black','FontSize',12)
                text(482,-15,['backspace = delete /'],'Color','black','FontSize',12)
                text(632,-15,['space = next image'],'Color','black','FontSize',12)
            
                text(0*imageSize(2)/10,imageSize(1)+20,['0 = ',classLabels(1)],'Color',classColors(1),'FontSize',12)
                text(1*imageSize(2)/10,imageSize(1)+20,['1 = ',classLabels(2)],'Color',classColors(2),'FontSize',12)
                text(2*imageSize(2)/10,imageSize(1)+20,['2 = ',classLabels(3)],'Color',classColors(3),'FontSize',12)
                text(3*imageSize(2)/10,imageSize(1)+20,['3 = ',classLabels(4)],'Color',classColors(4),'FontSize',12)
                text(4*imageSize(2)/10,imageSize(1)+20,['4 = ',classLabels(5)],'Color',classColors(5),'FontSize',12)
                text(5*imageSize(2)/10,imageSize(1)+20,['5 = ',classLabels(6)],'Color',classColors(6),'FontSize',12)
                text(6*imageSize(2)/10,imageSize(1)+20,['6 = ',classLabels(7)],'Color',classColors(7),'FontSize',12)
                text(7*imageSize(2)/10,imageSize(1)+20,['7 = ',classLabels(8)],'Color',classColors(8),'FontSize',12)
                text(8*imageSize(2)/10,imageSize(1)+20,['8 = ',classLabels(9)],'Color',classColors(9),'FontSize',12)
                text(9*imageSize(2)/10,imageSize(1)+20,['9 = ',classLabels(10)],'Color',classColors(10),'FontSize',12)
                
                %Delete last annotation, plot and save the other ones
                [numAnno,dummy] = size(tempAnnotations);
                 if numAnno > 0
                    tempAnnotations(end,:) = [];
                    [numAnno,dummy] = size(tempAnnotations);
                    if numAnno > 0
                        for k=1:numAnno
                            cColor = tempAnnotations(k,1);
                            pNum = tempAnnotations(k,2);
                            f = zeros(1,pNum);
                            for l = 1:pNum
                                f(1,l) = l;
                            end
                            position = zeros(pNum,2);
                            position(:,1) = tempAnnotations(k,3:pNum+2);
                            position(:,2) = tempAnnotations(k,pNum+3:2*pNum+2);
                            patch('Faces',f,'Vertices',position,'EdgeColor',classColors(cColor+1),'FaceColor','none','LineWidth',3);

                            %Save positions in Annotations matrix
                            z = zeros(1,2*maxNumPoints+2);
                            Annotations = [Annotations; z];
                            Annotations(end,1) = tempAnnotations(k,1);
                            Annotations(end,2) = tempAnnotations(k,2);
                            Annotations(end,3:pNum+2) = tempAnnotations(k,3:pNum+2);
                            Annotations(end,pNum+3:2*pNum+2) = tempAnnotations(k,pNum+3:2*pNum+2);
                        end
                    end
                 end
                 validKey = true;
               
            %Space will load next image
            elseif strcmp(key,'space')                 
                %If impoly is not zero, save positions to Annotations matrix.
                %Takes default class (0)
                if h ~= 0
                    position = getPosition(h);
                    
                    if(drawingStyle == 0)
                        %Polygon is drawn -> no manipulation has to be done

                    elseif(drawingStyle == 1) %Ellipse is drawn
                        xMid = position(1) + position(3)/2;
                        yMid = position(2) + position(4)/2;

                        a = position(3)/2;
                        b = position(4)/2;

                        position = [];

                        for ellipsAngle = 0:ellipsdisc:2*pi
                            xTemp = xMid + a*cos(ellipsAngle);
                            yTemp = yMid + b*sin(ellipsAngle);

                            position = [position; xTemp yTemp];
                        end

                    elseif(drawingStyle == 2) %Rectangle is drawn
                        positionTemp = position;
                        position = zeros(4,2);
                        position(1,1) = positionTemp(1);
                        position(1,2) = positionTemp(2);
                        position(2,1) = positionTemp(1);
                        position(2,2) = positionTemp(2) + positionTemp(4);
                        position(3,1) = positionTemp(1) + positionTemp(3);
                        position(3,2) = positionTemp(2) + positionTemp(4);
                        position(4,1) = positionTemp(1) + positionTemp(3);
                        position(4,2) = positionTemp(2);

                    else
                        error('drawingStyle not valid')
                    end
                    
                    if length(position) <= maxNumPoints
                        f = zeros(1,length(position));
                        z = zeros(1,2*maxNumPoints+2);
                        Annotations = [Annotations; z];
                        Annotations(end,1) = defaultClass;
                        Annotations(end,2) = length(position);
                        for k = 1:length(position)
                            f(1,k) = k;
                            
                            %Save positions in Annotations matrix
                            Annotations(end, k+2) = position(k,1);   %x values
                            Annotations(end, k+2+length(position)) = position(k,2); %y values
                        end
                    else
                        disp('ERROR: Too many points -> Increase variable "maxNumPoints"')
                        delete(h)
                    end
                end
                
                %Set validKey and nextImage to true --> next image can be shown
                validKey = true;
                nextImage = true;
                
            end
        end
        
    end
    
    %Save annotations in xml file
    imageNode = annotationsNode.createElement('Image');
    imageNode.setAttribute('name',imageList(i));
    
    %Iterate over all annotations of current image
    nm = size(Annotations);
    for j=1:nm(1)
        
        %Create new annotation node
        annotationNode = annotationsNode.createElement('Annotation');
        % floor rounds the number to the next smaller integer
        % ceil rounds the number to the next larger integer
        annotationNode.setAttribute('class',num2str(Annotations(j,1)));
        annotationNode.setAttribute('pointNum',num2str(Annotations(j,2)));
        pointNum = str2num(annotationNode.getAttribute('pointNum'));
        annotationNode.setAttribute('x',num2str(Annotations(j,3:pointNum+2)));
        annotationNode.setAttribute('y',num2str(Annotations(j,pointNum+3:2*pointNum+2)));
        %x_test = str2num(annotationNode.getAttribute('x'));
        %y_test = str2num(annotationNode.getAttribute('y'));
       
        %Append annotation node to image node
        imageNode.appendChild(annotationNode);
        
    end
    
    if nodeExists
        
        annotationsRootNode.replaceChild(imageNode, imageNodes.item(i-1));
        
    else
        %Append image node to root node
        annotationsRootNode.appendChild(imageNode);
    end
    
    %Set next image and write xml file
    annotationsRootNode.setAttribute('nextImage',num2str(i+1));
    xmlwrite(pathAnnotations,annotationsNode);
end

%Close figure
close(imageFigure);
disp('Finished annotating!');
