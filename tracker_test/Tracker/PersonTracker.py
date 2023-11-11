import numpy as np
import cv2
import Tracker.TrackedObject
import Tracker.TrackedDistance
import sacn
import time

# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
cv2.startWindowThread()
TrackedObjects = []

# Open webcam video stream
cap = cv2.VideoCapture('test.mp4')
# cap = cv2.VideoCapture(0)

# Constants
font = cv2.FONT_HERSHEY_SIMPLEX
destination_ip = "10.0.0.4"
obj_id = 1
area_threshold = 1000
max_p_age = 1000
distance_threshold = 30
double_resolution = True

crop_left_offset = 0
crop_right_offset = 0
crop_top_offset = 120
crop_bottom_offset = 90

light_top_left_x = 10
light_top_left_y = 40
light_bottom_right_x = 310
light_bottom_right_y = 320

light_top_left_pan = 10
light_top_left_tilt = 10
light_bottom_right_pan = 200
light_bottom_right_tilt = 300

sender = sacn.sACNsender(bind_address=destination_ip, source_name="NSYNC_Person_Tracker")
sender.start()
sender.activate_output(1)

dmx_data = []

while cap.isOpened():
    print(obj_id - 1)
    # Capture frame-by-frame
    ret, frame = cap.read()

    if frame is None:
        break

    for obj in TrackedObjects:
        # age every person one frame
        obj.age_one()

        if obj.timed_out():
            # get out of the people list
            index = TrackedObjects.index(obj)
            TrackedObjects.pop(index)
            del obj  # free the memory of i

    # resizing for faster detection. Crop image to remove extraneous pixels.
    frame = cv2.resize(frame, (640, 480))
    frame = frame[crop_top_offset:(480 - crop_bottom_offset), crop_left_offset:(640 - crop_right_offset)]

    # detect people in the image. Returns the bounding boxes for the detected objects
    boxes, weights = hog.detectMultiScale(frame, winStride=(6, 6), padding=(2, 2), scale=1.05, hitThreshold=0.5)
    rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])
    distances = []

    for (xA, yA, xB, yB) in rects:
        # display the detected boxes in the colour picture
        # cv2.rectangle(frame, (xA, yA), (xB, yB), (0, 255, 0), 2)
        rect = (xA, yA, xB, yB)
        area = (xA - xB) * (yA - yB)

        # if area > area_threshold:
            # Tracking algorithm

        # find center points
        cx = int(((xB - xA) / 2) + xA)
        cy = int(((yB - yA) / 3) + yA)

        for obj in TrackedObjects:
            d = Tracker.TrackedDistance.TrackedDistance(obj.get_x(), obj.get_y(), cx, cy)

            if d.dist < distance_threshold:
                distances.append(d)

        distances.sort(key=lambda x: x.dist)

        for distance in distances:
            if distance.dist < distance_threshold:
                # object recognized and within threshold
                for obj in TrackedObjects:
                    if obj.same_coords(distance.x1, distance.y1):
                        obj.update_coords(distance.x2, distance.y2)
            else:
                # new/unrecognized object detected
                t = Tracker.TrackedObject.TrackedObject(obj_id, cx, cy, max_p_age)
                TrackedObjects.append(t)
                obj_id += 1

        if distances.__len__() == 0:
            # new/unrecognized object detected
            t = Tracker.TrackedObject.TrackedObject(obj_id, cx, cy, max_p_age)
            TrackedObjects.append(t)
            obj_id += 1

        # if abs(cx - i.get_x()) <= (xB - xA) and abs(cy - i.get_y()) <= (yB - yA):
        #     # the object is close to one that has already been detected before
        #     new = False
        #
        #     # update coordinates in the object and resets age
        #     i.update_coords(cx, cy)

        # Draw boxes and center points
        img = cv2.rectangle(frame, (xA, yA), (xB, yB), (0, 255, 0), 2)

    for obj in TrackedObjects:
        cv2.circle(frame, (obj.get_x(), obj.get_y()), 5, (0, 0, 255), -1)
        cv2.putText(frame, str(obj.get_id()), (obj.get_x() + 5, obj.get_y()), font, 0.6, (0, 0, 255), 1, cv2.LINE_AA)

    # Generate DMX signals from person coordingates
    # TODO: expand to generate dmx for more than one person
    x_ratio = (TrackedObjects[0].get_x() - light_top_left_x) / (light_bottom_right_x - light_top_left_x)
    y_ratio = (TrackedObjects[0].get_y() - light_top_left_y) / (light_bottom_right_y - light_top_left_y)
    dmx_pan = int(x_ratio * (light_bottom_right_pan - light_top_left_pan)) + light_top_left_pan
    dmx_tilt = int(y_ratio * (light_bottom_right_tilt - light_top_left_tilt)) + light_top_left_tilt

    if double_resolution:
        dmx_data[0] = dmx_pan >> 8
        dmx_data[1] = dmx_pan % 255
        dmx_data[2] = dmx_tilt >> 8
        dmx_data[3] = dmx_tilt % 255
    else:
        dmx_data[0] = dmx_pan
        dmx_data[1] = dmx_tilt

    sender[1].dmx_data = tuple(dmx_data)

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()

# finally, close the window
cv2.destroyAllWindows()
cv2.waitKey(1)
