from djitellopy import tello
from time import sleep
import cv2
from openai import OpenAI
from pytesseract import Output
import pytesseract
pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'


# client = OpenAI(api_key= )

# # Define function to get chatbot response
# def get_chatbot_response(prompt):
#    chat_completion = client.chat.completions.create(
#     messages=[
#         {
#             "role": "user",
#             "content" : prompt,
#         }
#     ],
#     model="gpt-3.5-turbo",
# )
#    print(chat_completion.choices[0].message.content)
#    return chat_completion.choices[0].message.content
# print(get_response("Who is the current president of the United States?"))

# Define function to get chatbot response for a given input string
def get_response(input_string):
    return get_chatbot_response(input_string)



def connect_to_drone():
    me = tello.Tello()
    me.connect()
    print(me.get_battery())
    return me

def takeoff_and_land(me):
   
    me.takeoff()
    landing = show_webcam_stream(me)
    sleep(0.5)
    if landing == False or landing is None:
        me.land()   
        
        
def move_direction(me, direction):
    if direction == "forward":
        me.move_forward(20)
    elif direction == "backward":
        me.move_back(20)
    elif direction == "left":
        me.move_left(20)
    elif direction == "right":
        me.move_right(20)
    elif direction == "up":
        me.move_up(20)
    elif direction == "down":
        me.move_down(20)
    elif direction == "cw":
        me.rotate_clockwise(30)
    elif direction == "ccw":
        me.rotate_counter_clockwise(30)
        
   

def show_webcam_stream(me):
    # Initialize the webcam (use 0: default camera)
    me.streamon()
    while True:
        frame = me.get_frame_read().frame
        frame = cv2.resize(frame, (360, 240))
       # Convert the image to gray scale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

      # Detecting text from the image
        data = pytesseract.image_to_data(gray, output_type=Output.DICT)
        
        
        move_forward_command = 0
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
                        move_forward_command += 1
                        if move_forward_command < 5:
                            move_direction(me, "forward")
                            time.sleep(0.5)
                        else:
                            me.land()
                            break
                    elif area > 21921:
                        cv2.putText(frame, "Move Backwards", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        time.sleep(0.5)
                    else:
                        cv2.putText(frame, "Stop", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        if text == "Land":
                            me.land()
                            return True
                        
        cv2.imshow('Webcam Live - Text Detection', frame)
      

        if cv2.waitKey(1) & 0xFF == ord('q'):
            me.land()
    
    # When everything is done, release the capture
    cap.release()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    me = connect_to_drone()
    
    takeoff_and_land(me)

1