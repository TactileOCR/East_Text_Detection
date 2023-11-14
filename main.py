from openai import OpenAI
import os
from dotenv import load_dotenv
import os
import sys
import numpy as np
import cv2
import time
from imutils.object_detection import non_max_suppression

# Load values from the .env file if it exists
load_dotenv()

# Configure OpenAI
key = os.getenv("OPENAI_API_KEY")

client = OpenAI(api_key=key)

# Define function to get chatbot response
def get_chatbot_response(prompt):
   chat_completion = client.chat.completions.create(
    messages=[
        {
            "role": "user",
            "content" : prompt,
        }
    ],
    model="gpt-3.5-turbo",
)
   print(chat_completion.choices[0].message.content)
   return chat_completion.choices[0].message.content

# Define function to get chatbot response for a given input string
def get_response(input_string):
    return get_chatbot_response(input_string)

def east_detect(image):
    layerNames = [
        "feature_fusion/Conv_7/Sigmoid",
        "feature_fusion/concat_3"]

    orig = image.copy()

    if len(image.shape) == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

    (H, W) = image.shape[:2]
    (newW, newH) = (320, 320)
    rW = W / float(newW)
    rH = H / float(newH)
    image = cv2.resize(image, (newW, newH))
    (H, W) = image.shape[:2]

    net = cv2.dnn.readNet("frozen_east_text_detection.pb")
    blob = cv2.dnn.blobFromImage(image, 1.0, (W, H), (123.68, 116.78, 103.94), swapRB=True, crop=False)
    start = time.time()
    net.setInput(blob)
    (scores, geometry) = net.forward(layerNames)
    (numRows, numCols) = scores.shape[2:4]
    rects = []
    confidences = []

    for y in range(0, numRows):
        scoresData = scores[0, 0, y]
        xData0 = geometry[0, 0, y]
        xData1 = geometry[0, 1, y]
        xData2 = geometry[0, 2, y]
        xData3 = geometry[0, 3, y]
        anglesData = geometry[0, 4, y]

        for x in range(0, numCols):
            if scoresData[x] < 0.5:
                continue

            (offsetX, offsetY) = (x * 4.0, y * 4.0)
            angle = anglesData[x]
            cos = np.cos(angle)
            sin = np.sin(angle)
            h = xData0[x] + xData2[x]
            w = xData1[x] + xData3[x]
            endX = int(offsetX + (cos * xData1[x]) + (sin * xData2[x]))
            endY = int(offsetY - (sin * xData1[x]) + (cos * xData2[x]))
            startX = int(endX - w)
            startY = int(endY - h)
            rects.append((startX, startY, endX, endY))
            confidences.append(scoresData[x])

    boxes = non_max_suppression(np.array(rects), probs=confidences)
    
    cropped_images = []  # List to hold cropped images

    for (startX, startY, endX, endY) in boxes:
        startX = int(startX * rW)
        startY = int(startY * rH)
        endX = int(endX * rW)
        endY = int(endY * rH)
        cv2.rectangle(orig, (startX, startY), (endX, endY), (0, 255, 0), 2)

        # Crop and save each detected region
        cropped_region = orig[startY:endY, startX:endX]
        cropped_images.append(cropped_region)

    print(time.time() - start)
    return orig

# Example usage
# image = cv2.imread("a-sign-big.jpg")
# detected_image, cropped_texts = east_detect(image)
# for i, cropped in enumerate(cropped_texts):
#     cv2.imshow(f"Cropped Text {i+1}", cropped)
#     text = read_image(cropped)
  
# # cv2.imshow("Detected Text", detected_image)
# # cv2.waitKey(0)
# # cv2.destroyAllWindows()


import pytesseract
from PIL import Image
def read_image(image):
    return pytesseract.image_to_string(image)

def add_to_records(records_dict, response, text):
    records_dict[text] = response
    return records_dict

def append_to_file(data_to_append):
    """
    Appends data to a file. Creates the file if it doesn't exist.

    Args:
    file_path (str): The path to the file.
    data_to_append (str): The data to append to the file.
    """
    try:
        # Open the file in append mode, which creates the file if it doesn't exist
        with open('response.txt', 'a') as file:
            file.write(data_to_append)
        print(f"Data appended to successfully.")
    except IOError as e:
        print(f"An error occurred while writing: {e}")


def image_processing(path_to_image):
    image = cv2.imread(path_to_image)
    out_image = east_detect(image)
    cv2.imwrite(f"sample_output-{path_to_image}.jpg", out_image)
    image = Image.open(path_to_image)
    text = read_image(image)
    return text

def main(path_to_image):
    records_dict = {}
    pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'
    text = image_processing(path_to_image)
    print(text)
    print(records_dict)
    if text not in records_dict:
        response = get_response(text)
        append_to_file(response)
        records_dict = add_to_records(records_dict, response, text)
        print(records_dict)
        
        
main('Capture.JPG')


