function[prompt, response] = OCRChatgpt() 
    model = 'craftNet.mat';
    I = imread(path_to_image);
    [image, imageScale]  = helper.preprocess(I);
    out = craftPredict(model,im2single(image));
    boundingBoxes = helper.postprocess(out,imageScale);
    outImg = insertShape(I,'rectangle',boundingBoxes,'LineWidth',5,'Color',"yellow");
    figure, imshow(outImg)
    ocrResults = ocr(I);
    text = ocrResults.Text;
    prompt = "What is "+ text;  
    response = chat5(prompt);
    disp(prompt)
    disp(response)
end
