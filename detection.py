import cv2
from sys import argv
import matplotlib.pyplot as plt
import numpy as np
from imutils.video import VideoStream
from imutils.video import FPS
import argparse
import imutils
import time

#set the minimum threshold value to avoid weak detections
threshold = 0.65;

# initialize the list of class labels MobileNet SSD finetuned by us was trained to
# detect, then generate a set of bounding box colors for each class
CLASSES = ["unlabeled", "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic", "fire", "street", "stop", "parking", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "hat", "backpack", "umbrella", "shoe", "eye", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports", "kite", "baseball", "baseball", "skateboard", "surfboard", "tennis", "bottle", "plate", "wine", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot", "pizza", "donut", "cake", "chair", "couch", "potted", "bed", "mirror", "dining", "window", "desk", "toilet", "door", "tv", "laptop", "mouse", "remote", "keyboard", "cell", "microwave", "oven", "toaster", "sink", "refrigerator", "blender", "book", "clock", "vase", "scissors", "teddy", "hair", "toothbrush", "hair", "banner", "blanket", "branch", "bridge", "building-other", "bush", "cabinet", "cage", "cardboard", "carpet", "ceiling-other", "ceiling-tile", "cloth", "clothes", "clouds", "counter", "cupboard", "curtain", "desk-stuff", "dirt", "door-stuff", "fence", "floor-marble", "floor-other", "floor-stone", "floor-tile", "floor-wood", "flower", "fog", "food-other", "fruit", "furniture-other", "grass", "gravel", "ground-other", "hill", "house", "leaves", "light", "mat", "metal", "mirror-stuff", "moss", "mountain", "mud", "napkin", "net", "paper", "pavement", "pillow", "plant-other", "plastic", "platform", "playingfield", "railing", "railroad", "river", "road", "rock", "roof", "rug", "salad", "sand", "sea", "shelf", "sky-other", "skyscraper", "snow", "solid-other", "stairs", "stone", "straw", "structural-other", "table", "tent", "textile-other", "towel", "tree", "vegetable", "wall-brick", "wall-concrete", "wall-other", "wall-panel", "wall-stone", "wall-tile", "wall-wood", "water-other", "waterdrops", "window-blind", "window-other", "wood"]

COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

# load our serialized model from disk
print("[INFO] loading model...")
net = cv2.dnn.readNet('frozen_inference_graph.bin', 'frozen_inference_graph.xml')

# specify the target device as the Myriad processor on the NCS
net.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)


# initialize the video stream, allow the cammera sensor to warmup,
# and initialize the FPS counter
print("[INFO] starting video stream...")
vs = cv2.VideoCapture(0)
vs.set(3, 1920)
vs.set(4, 1080)
time.sleep(2.0)
fps = FPS().start()
x_center = 0
y_center = 0

# loop over the frames from the video stream
while True:
        # grab the frame from the threaded video stream, crop a part of it and resize it
        # to have a maximum width of 300 pixels, then rotate it 90 degrees
        ret, frame = vs.read()
        cropped = frame[60:1080-60, 0:960]
        cropped = imutils.resize(cropped, width=300)
        cropped = imutils.rotate(cropped, 90)
		
        # grab the frame dimensions and convert it to a blob
        (h, w) = cropped.shape[:2]
        frame = cropped
        blob = cv2.dnn.blobFromImage(cropped, size=(300,300), ddepth=cv2.CV_8U)  
		
        # pass the blob through the network and obtain the detections and predictions
        net.setInput(blob)
        detections = net.forward()
		
		#reshape the numpy array that contains the detections
        detections2 = detections.reshape(-1,7)
		
        # loop over the detections
        for detection in detections2:
            # extract the confidence (i.e., probability) associated with
            # the prediction
            confidence = float(detection[2])
            # filter out weak detections by ensuring the `confidence` is
            # greater than the minimum confidence
            if confidence > threshold:
                # extract the index of the class label from the
                # `detections`, then compute the (x, y)-coordinates of
                # the bounding box for the object
                idx = int(detection[1])
                box = detection[3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
				#if the detected object is a type of recyclable trash, write the coordinates of it in a .txt file
                if CLASSES[idx] == "bottle":
                    x_center = (startX+endX)/2
					
                    file = open("testfile.txt","w")
                    file.seek(0)                        
                    file.truncate()
                    file.write(str(x_center)+","+str(endY))
                    file.close()
                # draw the prediction on the frame
                label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
                cv2.rectangle(frame, (startX, startY), (endX, endY), COLORS[idx], 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
                        
        # show the output frame    
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break

        # update the FPS counter
        fps.update()
# stop the timer and display FPS information
fps.stop()
print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()