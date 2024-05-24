import cv2
import pytesseract
from pytesseract import Output
import shutil
from dotenv import load_dotenv

# Load values from the .env file if it exists
load_dotenv()

#pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'
pytesseract.pytesseract.tesseract_cmd = shutil.which("tesseract") 

def show_webcam_stream():
    # Initialize the webcam (use 0: default camera)
    cap = cv2.VideoCapture(0)

    # Check if the webcam is opened correctly
    if not cap.isOpened():
        raise IOError("Cannot open webcam")
    
    
    centerpoint = (320 // 2, 320 // 2)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        frame = cv2.resize(frame, (360, 240))
       # Convert the image to gray scale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

      # Detecting text from the image
        data = pytesseract.image_to_data(gray, output_type=Output.DICT)
        
        
        
        # Draw bounding boxes around text
        n_boxes = len(data['level'])
        found_areas = []
        info = []
        for i in range(n_boxes):
            if data["text"][i] != "" or data["text"][i].strip() != "" or data["text"][i].strip('\n') != "" and data['left'][i] > 0 and data['top'][i] > 0 and data['width'][i] != 0 and data['height'][i] != 0  and data['left'][i] + data['width'][i] < 360 and data['left'][i] + data['top'][i] < 240 :
                if data['conf'][i] >= 93:
                    (x, y, w, h) = (data['left'][i], data['top'][i], data['width'][i], data['height'][i])
                    frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    # cv2.putText(frame, data["text"][i], (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    cx = x  + w // 2
                    cy = y + h // 2
                    area = w * h
                    cv2.circle(frame, (cx, cy) , 5, (0, 0, 255), cv2.FILLED)
                    if area < 21000:
                        cv2.putText(frame, "Move Forward", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    elif area > 21921:
                        cv2.putText(frame, "Move Backwards", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    else:
                        cv2.putText(frame, "Stop", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
        cv2.imshow('Webcam Live - Text Detection', frame)
      

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # When everything is done, release the capture
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    show_webcam_stream()
