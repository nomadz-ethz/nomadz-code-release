%% Tabula Rasa
clc
clear all
close all

%% Copy files
folder = 'acquisition_ETH_17_06_27';
listPath = strcat(folder, '/input');
inputPath = strcat(folder, '/xml_complete');
outputPath = strcat(folder, '/xml');

fileList = dir(listPath);

l = length(fileList);

for i = 3:l
    file = fileList(i).name;
    file = strrep(file,'.bmp','');
    file = strcat(file,'.xml');
    file = strcat(inputPath,'/',file);
    copyfile(file,outputPath);
end

disp('Completed!')
