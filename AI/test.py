import cv2
import numpy as numpy
import time

class Timestamp:
    def __init__(self):
        self.people = -1
        self.time = None

cap = cv2.VideoCapture(0)

MAX_PERSONS = 1
ROLLING_TIME_PERIOD = 60

target_width = 320
target_height = 320

classes = ["person"]
model_config = "yolov3-tiny.cfg"
model_weights = "yolov3-tiny.weights"

net = cv2.dnn.readNetFromDarknet(model_config, model_weights)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

def is_similar(bbox1, bbox2):
    first_x1, first_y1, first_x2, first_y2 = bbox1
    second_x1, second_y1, second_x2, second_y2 = bbox2

    first_point_distance = ((first_x1 - second_x1) ** 2 + (first_y1 - second_y1) ** 2) ** 0.5
    second_point_distance = ((first_x2 - second_x2) ** 2 + (first_y2 - second_y2) ** 2) ** 0.5

    return first_point_distance + second_point_distance <= 200

def is_valid_person(detection):
    return detection[5] >= 0.1

def is_at_capacity(window):
    sum = 0
    for timestamp in window:
        sum += timestamp.people
    average = float(sum) / len(window)
    average_str = str(average)
    decimals = average_str.split(".")[-1]
    print(average)
    if decimals[0] == "9":
        average += 1
    return average >= MAX_PERSONS

def remove_outdated_timestamps(window):
    now = time.time()
    new_window = []
    for timestamp in window:
        if int(now - timestamp.time) < ROLLING_TIME_PERIOD:
            new_window.append(timestamp)
        else:
            print("Removing timestamp")
    return new_window
        

window = []

while True:
    success, img = cap.read()

    image_height, image_width, _ = img.shape

    blob = cv2.dnn.blobFromImage(img, 1/255, (target_width, target_height), [0, 0, 0], 1, crop=False)
    net.setInput(blob)

    layer_names = net.getLayerNames()
    output_layer_indexes = net.getUnconnectedOutLayers()
    output_layer_names = [layer_names[output_layer_indexes[0][0] - 1],
                          layer_names[output_layer_indexes[1][0] - 1]]

    output = net.forward(output_layer_names)

    valid_bboxes = []
    for network_output in output:
        for detection in network_output:
            if is_valid_person(detection):
                width, height = int(detection[2] * image_width), int(detection[3] * image_height)
                starting_x = int((detection[0] * image_width) - (width / 2))
                starting_y = int((detection[1] * image_height) - (height / 2))

                valid_bboxes.append((starting_x, starting_y, starting_x + width, starting_y + height))

    new_valid_bboxes = []
    for bbox in valid_bboxes:
        similar = False
        for known_bbox in new_valid_bboxes:
            if is_similar(bbox, known_bbox):
                similar = True
                break
        if not similar:
            new_valid_bboxes.append(bbox)


    number_of_people = len(new_valid_bboxes)
    timestamp = Timestamp()
    timestamp.people = number_of_people
    timestamp.time = time.time()

    window.append(timestamp)

    if is_at_capacity(window):
        # THROW THE ERROR
        print("AT CAPACITY")

    window = remove_outdated_timestamps(window)
