import cv2
import numpy as np

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

def draw_circles(frame):
    frame = cv2.circle(frame, (335,167), radius=0, color=(0, 255, 0), thickness=5)
    frame = cv2.circle(frame, (335,270), radius=0, color=(0, 255, 0), thickness=5)
    frame = cv2.circle(frame, (435,167), radius=0, color=(0, 255, 0), thickness=5)
    frame = cv2.circle(frame, (435,270), radius=0, color=(0, 255, 0), thickness=5)
    return frame

def main():
    # global hand_hist
    # is_hand_hist_created = False
    capture = cv2.VideoCapture(2)

    while True:
        # pressed_key = cv2.waitKey(0)

        _, frame = capture.read()
        # frame = draw_rect(frame)
        # frame = cv2.circle(frame, (30,10), radius=0, color=(0, 0, 255), thickness=10)
        frame = draw_circles(frame)
        cv2.imshow("Guides", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # while capture.isOpened():
    #     pressed_key = cv2.waitKey(1)
    #     _, frame = capture.read()
    #     frame = cv2.flip(frame, 1)

    #     if pressed_key & 0xFF == ord('z'):
    #         is_hand_hist_created = True
    #         hand_hist = hand_histogram(frame)

    #     if is_hand_hist_created:
    #         manage_image_opr(frame, hand_hist)

    #     else:
    #         frame = draw_rect(frame)

    #     cv2.imshow("Live Feed", rescale_frame(frame))

    #     if pressed_key == 27:
    #         break

    # After the loop release the cap object
    capture.release()
    # Destroy all the windows
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main()