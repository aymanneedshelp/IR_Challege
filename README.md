**annotation_pub.py**
This file is for starting the Computer Vision. It is to be run on the Turtlebot and will publish analysed images to /annotated_frames and corresponding labels to /trigger (named as such since it is triggered on a new image detection)

**annotation_subsciber.py**
This file is for subscribing to the /annotated_frames topic and displaying the annotated images on the remote computer.

**label_subscriber.py**
This file implements the counter functionality. The **annotation_pub.py** file takes care of sending only unique images, by analysing the label of the previously annotated image and the location in which it was analysed. This file implements a dictionary to count the number of occurences for each captured image.
