# import the opencv library
#!/usr/bin/env python
import cv2

def change_res(cap,width, height):
    cap.set(3, width)
    cap.set(4, height)

def rescale_frame(frame, percent=75):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)

# define a video capture object
vid = cv2.VideoCapture(1)
cv2.namedWindow("Display", cv2.WINDOW_NORMAL)
while(True):
      
    # Capture the video frame
    # by frame
    ret, frame = vid.read()
    # print(ret,frame.shape)
    # Display the resulting frame
    frame75 = rescale_frame(frame, percent=25)
    print(ret,frame75.shape)

    cv2.imshow('Display', frame75)
      
    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()