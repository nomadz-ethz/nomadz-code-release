%% Tabula Rasa
clc
clear all
close all

%% Read in image list
%Paths
folder = 'test';
pathImages = strcat(folder,'/input'); %Path to images
pathOutputIncreased = strcat(folder,'/increased');
pathOutputDecreased = strcat(folder,'/decreased');

%Image list
imageList = dir(pathImages);
imageList = imageList(arrayfun(@(x) ~strcmp(x.name(1),'.'),imageList));
imageList = {imageList(~[imageList.isdir]).name};
sort(imageList);
l = length(imageList);

mFactor = 0.3333;
aFactor = 50;

for i = 1:2
    if(i == 1)
        multFactor = 1-mFactor; %decrease brightness
        addFactor = -aFactor;
        pathOutput = pathOutputDecreased;
    elseif(i == 2)
        multFactor = 1+mFactor; %increase brightness
        addFactor = aFactor;
        pathOutput = pathOutputIncreased;
    end
    
    for j = 1:l
        imageNameTemp = imageList{j};
        imagePathTemp = strcat(pathImages, '/', imageNameTemp);
        imageTemp = imread(imagePathTemp);
%         newImageTemp = multFactor*imageTemp;
        newImageTemp = imageTemp + addFactor;
        newImagePathTemp = strcat(pathOutput, '/', imageNameTemp);
        imwrite(newImageTemp, newImagePathTemp);
    end
end

disp('Finished')