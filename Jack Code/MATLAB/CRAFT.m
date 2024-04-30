%% Code Generation for CRAFT
% The following code demonstrates code generation for a pre-trained 
% CRAFT text detection network.

%% Setup
% Add path to the source directory.
mypi = raspi('myPi');
cam = cameraboard(mypi,'Resolution','640x480');

addpath('src');
for k = 1:1000
orgImage = snapshot(cam);

% Preprocess the image
[image, imageScale] = helper.preprocess(orgImage);

% % Provide location of the mat file of the trained network
matFile = 'craftNet.mat';

% Call craft_predict_mex on the input image
out = craftPredict(matFile,im2single(image));

% apply post-processing on the output
boundingBoxes = helper.postprocess(out,imageScale);

% Visualize results
outImg = insertShape(orgImage,'Polygon',boundingBoxes,'LineWidth',5,'Color',"yellow");

image(outImg);
drawnow;

end


