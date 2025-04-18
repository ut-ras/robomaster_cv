# References:
# https://docs.opencv.org/4.x/dd/d43/tutorial_py_video_display.html
# https://docs.opencv.org/4.x/d9/dc8/tutorial_py_trackbar.html
# https://docs.opencv.org/4.x/d7/d4d/tutorial_py_thresholding.html
# https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html
# https://docs.opencv.org/4.5.4/da/d97/tutorial_threshold_inRange.html
# https://docs.opencv.org/4.5.4/db/df6/tutorial_erosion_dilatation.html
# https://docs.opencv.org/3.4/d4/d73/tutorial_py_contours_begin.html
# https://docs.opencv.org/4.x/dc/da5/tutorial_py_drawing_functions.html
# https://stackoverflow.com/questions/38064777/use-waitkey-in-order-pause-and-play-video
# https://docs.opencv.org/4.x/de/d62/tutorial_bounding_rotated_ellipses.html
# https://www.geeksforgeeks.org/python-opencv-cv2-puttext-method/
#
import cv2 as cv
import numpy as np
import math
import time
import os
import sys
import sort 



try:
    color = sys.argv[1]
    if (color == "red"):
        video_name = "tracking2.mp4"
    else:
        video_name = "tracking1.mp4"
except IndexError:
    color = 'red'
    video_name = "tracking2.mp4"

# Measures how similar two numbers are
def sim(a, b):
    return min(a, b) / (a + b)

def dist_sq(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2

def angle(a, b):
    if a[0] == b[0]:
        return 90
    right = a
    left = b
    if b[0] > a[0]:
        right = b
        left = a
    return math.degrees(math.atan((right[1] - left[1]) / (right[0] - left[0])))

def get_contours(frame, color):
    # Apply a red mask to image, apply morphological opening/closing, and find contours of contiguous red areas
    frame_HSV = None

    mask1, mask2, frame_threshold = None, None, None
    if color == 'red':
        frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask1 = cv.inRange(frame_HSV, (0, 70, 50), (20, 255, 255))
        mask2 = cv.inRange(frame_HSV, (170, 70, 50), (230, 255, 255))
        frame_threshold = mask1 | mask2
    else:
        frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask1 = cv.inRange(frame_HSV, (90, 100, 100), (115, 255, 255))
        mask2 = cv.inRange(frame_HSV, (115, 100, 100), (135, 255, 255))
        frame_threshold = mask1 | mask2

    frame_threshold = cv.erode(frame_threshold, cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5)))
    frame_threshold = cv.dilate(frame_threshold, cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5)))

    frame_threshold = cv.dilate(frame_threshold, cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5)))
    frame_threshold = cv.erode(frame_threshold, cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5)))

    contours, _ = cv.findContours(frame_threshold, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    return contours

def draw_centers(frame, color, detections: list[tuple[float, float]]):
    contours = get_contours(frame, color)

    # Compute rotated bounding box for each contour and store in `bboxes`
    bboxes = []
    for contour in contours:
        bbox = cv.minAreaRect(contour)
        bboxes.append(bbox)
        bbox_points = cv.boxPoints(bbox)
        bbox_points = np.intp(bbox_points)
        # frame = cv.drawContours(frame, [bbox_points], -1, (0, 255, 0), 2)

    thresh = 20
    width_sim_thresh, length_sim_thresh, y_thresh, angle_thresh = 0.1, 0.3, 0.15, 15
    for i in range(len(bboxes)):
        bbox1 = bboxes[i]
        center1 = (bbox1[0][0], bbox1[0][1])
        width1, length1, angle1 = bbox1[1][0], bbox1[1][1], bbox1[2]

        vert1 = cv.boxPoints(bbox1)

        longest = (0, 1)
        bl = vert1[0]
        tl = vert1[1]
        br = vert1[3]
        width1 = math.sqrt(dist_sq(vert1[0], vert1[3]))
        if dist_sq(bl, br) > dist_sq(bl, tl):
            longest = (0, 3)
            width1 = math.sqrt(dist_sq(vert1[0], vert1[1]))
        length1 = math.sqrt(dist_sq(vert1[longest[0]], vert1[longest[1]]))
        angle1 = angle(vert1[longest[0]], vert1[longest[1]])

        # If the bounding box is too small, skip
        if max(length1, width1) < thresh:
            continue

        # how the heck is `minAreaRect` defining the angle
        # if angle1 > 45:
        #     angle1 = 90 - angle1
        #     width1, length1 = length1, width1

        # Matching longer sides is more important, and thus needs a stricter threshold
        # if width1 > length1:
        #     width_sim_thresh, length_sim_thresh = length_sim_thresh, width_sim_thresh

        for j in range(i + 1, len(bboxes)):
            bbox2 = bboxes[j]
            center2 = (bbox2[0][0], bbox2[0][1])
            width2, length2, angle2 = bbox2[1][0], bbox2[1][1], bbox2[2]

            if max(length2, width2) < thresh:
                continue

            vert2 = cv.boxPoints(bbox2)
            
            longest = (0, 1)
            bl = vert2[0]
            tl = vert2[1]
            br = vert2[3]
            width2 = math.sqrt(dist_sq(vert2[0], vert2[3]))
            if dist_sq(bl, br) > dist_sq(bl, tl):
                longest = (0, 3)
                width2 = math.sqrt(dist_sq(vert2[0], vert2[1]))
            length2 = math.sqrt(dist_sq(vert2[longest[0]], vert2[longest[1]]))
            angle2 = angle(vert2[longest[0]], vert2[longest[1]])

            # if angle2 > 45:
            #     angle2 = 90 - angle2
            #     width2, length2 = length2, width2

            angle_diff = abs(angle1 - angle2)
            y_diff = abs(center1[1] - center2[1])

            # If two bounding boxes are similar in size and orientation, place a dot between them
            if width_sim_thresh < sim(width1, width2) \
                and length_sim_thresh < sim(length1, length2) \
                and y_diff < y_thresh * (length1 + length2) / 2 \
                and (angle_diff < angle_thresh or angle_diff > 180 - angle_thresh):
                cv.circle(frame, (round((bbox1[0][0] + bbox2[0][0]) / 2), round((bbox1[0][1] + bbox2[0][1]) / 2)), 10, (255, 0, 255), -1)


                detections.append((round((bbox1[0][0] + bbox2[0][0]) / 2), round((bbox1[0][1] + bbox2[0][1]) / 2)))
                
            # cv.putText(frame, f'w: {sim(width1, width2):.2f}, l: {sim(length1, length2):.2f}, a1: {angle1:.2f}, a2: {angle2:.2f}, {angle_diff:.2f}', (round((bbox1[0][0] + bbox2[0][0]) / 2), round((bbox1[0][1] + bbox2[0][1]) / 2)), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv.LINE_AA)

            # Debugging
            # print(f'w --- {i}: {width1}, {j}: {width2}, sim: {sim(width1, width2)}')
            # print(f'l --- {i}: {length1}, {j}: {length2}, sim: {sim(length1, length2)}')
            # print(f'a --- {i}: {angle1}, {j}: {angle2}, diff: {angle_diff}')

            # print((round((bbox1[0][0] + bbox2[0][0]) / 2), round((bbox1[0][1] + bbox2[0][1]) / 2)))

    # Reference square to see size of `thresh`
    # cv.rectangle(frame, (50, 50), (50 + thresh, 50 + thresh), (255, 0, 255), 1)

    return frame

def main():
    cap = cv.VideoCapture(video_name)
    tracker = sort.Sort()
    size = 50

    while cap.isOpened():
        detections = []

        ret, frame = cap.read()
        if not ret:
            print('Failed to read frame. Exiting...')
            break

        frame = draw_centers(frame, color, detections)

        detection_boxes = []
        for (x, y) in detections:
            detection_boxes.append([x - size, y - size, x + size, y + size, 0.3])

        detection_boxes = np.array(detection_boxes)
        if (detection_boxes.size != 0):
            tracked_objects, predicted_points = tracker.update(detection_boxes)
        else:
            tracked_objects, predicted_points = tracker.update(np.empty((0, 5)))

        frame_number = int(cap.get(cv.CAP_PROP_POS_FRAMES)) # From chatgpt
        # print(f"\nFrame {frame_number}:")

        # draw predicted points
        if (predicted_points.size != 0):
            for point in predicted_points:
                x1, y1, x2, y2, point_id = point.astype(int)
                cv.circle(frame, (x1 + size, y1 + size), 10, (0, 255, 0), -1)

        for track in tracked_objects:
            x1, y1, x2, y2, track_id = track.astype(int)
            print(f"ID {track_id}: Position ({x1 + size}, {y1 + size})")
            cv.putText(frame, 
                           f"{track_id}", 
                           (x1 + 50, y1 + 20), 
                           cv.FONT_HERSHEY_SIMPLEX, 
                           1, 
                           (255, 255, 255), 
                           1, cv.LINE_AA)


        cv.imshow('frame', frame)
        sys.stdout.flush()
        if cv.waitKey(50) == ord('q'):
            break
    cap.release()
    cv.destroyAllWindows()

main()