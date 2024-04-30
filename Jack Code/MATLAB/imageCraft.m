function imageCraft()
mypi = raspi('myPi');
cam = cameraboard(mypi,'Resolution','640x480');    
matFile = 'craftNet.mat';
addpath('src');
for k = 1:100
orgImage = snapshot(cam);

% Preprocess the image
[image, imageScale] = helper.preprocess(orgImage);

% Call craft_predict_mex on the input image
out = craftPredict(matFile,im2single(image));

% apply post-processing on the output
boundingBoxes = helper.postprocess(out,imageScale);





%OCR process
grayImage = rgb2gray(orgImage);
Ibinary = imbinarize(grayImage);
textResult = ocr(Ibinary);
recognizedText = textResult.Text;

% Visualize results
outImg = insertShape(orgImage,'Polygon',boundingBoxes,'LineWidth',5,'Color',"yellow");
imshow(outImg);

if ~isempty(recognizedText)
disp(recognizedText);
disp(textResult.TextLineConfidences);
end
drawnow;


end

% prompt = "What is "+ text;  
% image(outImg);
% drawnow;
% disp(text);
% disp(prompt)
% disp(response)
end
