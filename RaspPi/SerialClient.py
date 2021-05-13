import cv2
import time
import socket
import serial
import numpy


PORT = 'COM3'
BUAD_RATE = 115200
MAX_PIXELS_TO_SEND = 2000

class SerialWrapper:
    def __init__(self, device):
        self.ser = serial.Serial(device, BUAD_RATE)

    def sendData(self, data):
        data += "\r\n".encode()
        # self.ser.write("data".encode())
        # data = get_sample_data()
        self.ser.write(data)


def get_sample_data():
    pixel_count = 12
    difference_data = b''
    for i in range(pixel_count):
        difference_data += (100 + i).to_bytes(1, "little") # height
        difference_data += (200 + i).to_bytes(1, "little") # width

    data = "-2".encode()
    data += str(len(difference_data) + 2).zfill(4).encode()
    data += difference_data
    data += "\r\n".encode()
    return data

def send_to_socket(serial, non_zero_difference):
    if non_zero_difference is not None:
        difference_data = b''
        if len(non_zero_difference) > MAX_PIXELS_TO_SEND:
            pixel_count = MAX_PIXELS_TO_SEND 
            numpy.random.shuffle(non_zero_difference)
        else:
            pixel_count = len(non_zero_difference)
        
        for iDiff in range(pixel_count):
            difference_data += non_zero_difference[iDiff][0][1] # height
            difference_data += non_zero_difference[iDiff][0][0] # width
        msg = "-2".encode()
        msg += str(len(difference_data) + 2).zfill(4).encode()
        msg += difference_data
        serial.sendData(msg)


def read_camera_data(serial):

    camera = cv2.VideoCapture(0)

    _, previous_image = camera.read()
    width, height, _ = previous_image.shape
    
    while True:

        start = time.time()
        _, current_image = camera.read()
        end = time.time()
        difference = cv2.absdiff(current_image, previous_image)
        difference = cv2.cvtColor(difference, cv2.COLOR_BGR2GRAY)
        thresh, difference = cv2.threshold(difference, 60, 255, cv2.THRESH_BINARY)

        non_zero_difference = cv2.findNonZero(difference)
        
        send_to_socket(serial, non_zero_difference)

        cv2.imwrite("test.png", difference)

        previous_image = current_image
        print(end - start)


if __name__ == '__main__':
    serial = SerialWrapper(PORT)
    read_camera_data(serial)

        
        

    