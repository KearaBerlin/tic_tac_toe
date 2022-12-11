# import the opencv library
import cv2


# hand_hist = None
# traverse_point = []
# total_rectangle = 9
# hand_rect_one_x = None
# hand_rect_one_y = None

# hand_rect_two_x = None
# hand_rect_two_y = None

# def draw_rect(frame):
#     rows, cols, _ = frame.shape
#     global total_rectangle, hand_rect_one_x, hand_rect_one_y, hand_rect_two_x, hand_rect_two_y

#     hand_rect_one_x = np.array(
#         [6 * rows / 20, 6 * rows / 20, 6 * rows / 20, 9 * rows / 20, 9 * rows / 20, 9 * rows / 20, 12 * rows / 20,
#          12 * rows / 20, 12 * rows / 20], dtype=np.uint32)

#     hand_rect_one_y = np.array(
#         [9 * cols / 20, 10 * cols / 20, 11 * cols / 20, 9 * cols / 20, 10 * cols / 20, 11 * cols / 20, 9 * cols / 20,
#          10 * cols / 20, 11 * cols / 20], dtype=np.uint32)

#     hand_rect_two_x = hand_rect_one_x + 10
#     hand_rect_two_y = hand_rect_one_y + 10

#     for i in range(total_rectangle):
#         cv2.rectangle(frame, (hand_rect_one_y[i], hand_rect_one_x[i]),
#                       (hand_rect_two_y[i], hand_rect_two_x[i]),
#                       (0, 255, 0), 1)

#     return frame



  
# define a video capture object
vid = cv2.VideoCapture("/dev/video0")
  
while(True):
      
    # Capture the video frame
    # by frame
    ret, frame = vid.read()
  
    # Display the resulting frame
    cv2.imshow('frame', frame)
      
    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
