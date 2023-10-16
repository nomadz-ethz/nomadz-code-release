% Rename Images
% ------------------

% Created by Simon Zimmermann, HS2017

%Paths
folder_rename = 'acqusition_17_06_24';
pathImages = strcat(folder_rename,'/input'); %Path to images
% pathXml = strcat(pathImages,'/xml'); %Path to xml files

%Get list and number of images
imageList = dir(pathImages);
imageList = imageList(arrayfun(@(x) ~strcmp(x.name(1),'.'),imageList));
imageList = {imageList(~[imageList.isdir]).name};

%List length
l = length(imageList);

%Rename images according to folder name
for i = 1:l
    %Get old name
    imageNameOld = strjoin(imageList(i));
%     nameOld = strrep(imageNameOld,'.bmp','');
%     xmlNameOld = strcat(nameOld,'.xml');
    
    %Get number
    num = num2str(i);
    if(i <= 9)
        nameNum = strcat('00', num);
    elseif(i > 9)
        nameNum = strcat('0', num);
    elseif(i > 99)
        nameNum = num;
    end
    
    %Get new name
%     nameNew = strcat(folder_rename, '_image_', nameNum); %for comparison
%     imageNameNew = strcat(folder_rename, '_image_', nameNum, '.bmp');
%     xmlNameNew = strcat(folder_rename, '_camera_', nameNum, '.xml');
    imageNameNew = strrep(imageNameOld,'0_','');
    
%     %Rename image and corresponding xml file
%     if(strcmp(nameOld, nameNew))
%         disp('Image already renamed!')
%         continue;
%     else
        movefile(strcat(pathImages,'/',imageNameOld), strcat(pathImages,'/',imageNameNew));
%         movefile(strcat(pathXml,'/',xmlNameOld), strcat(pathXml,'/',xmlNameNew));
%     end
end

disp('Renaming completed!')