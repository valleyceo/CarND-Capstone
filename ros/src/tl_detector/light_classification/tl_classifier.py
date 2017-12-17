from styx_msgs.msg import TrafficLight
import sys
import os
import tensorflow as tf
import numpy as np
import rospy
import time

# trained model path
MODEL_PATH = os.path.join(os.getcwd(), '../../..', 'train_classifier')

# add tensorflow models path (https://github.com/tensorflow/models)
ROOT_PATH = os.path.join(os.getcwd(), '../../..')
sys.path.append(os.path.join(ROOT_PATH, 'models/research/'))
sys.path.append(os.path.join(ROOT_PATH, 'models/research/object_detection/utils'))

# import tensorflow models functions
from label_map_util import load_labelmap, convert_label_map_to_categories, create_category_index
from visualization_utils import visualize_boxes_and_labels_on_image_array

##################################
# Traffic Light Classifier Class #
##################################
class TLClassifier(object):

    def __init__(self):
        #TODO load classifier
        model_path = os.path.join(MODEL_PATH, "frozen_inference_graph.pb")
        label_path = os.path.join(MODEL_PATH, "train_data/label_map.pbtxt")
        NUM_CLASSES = 4
        
        label_map = load_labelmap(label_path)
        categories = convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
                                                                    use_display_name=True)
        self.category_index = create_category_index(categories)
        
        #### Build network
        self.image_np_deep = None
        self.detection_graph = tf.Graph()

        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True

        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(model_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=self.detection_graph, config=config)

        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        
        run_network = True

        # For timing single frame detection
        #start = time.time()
        
        if run_network is True:
            image_np_expanded = np.expand_dims(image, axis=0)

            # Actual detection.
            with self.detection_graph.as_default():
                (boxes, scores, classes, num) = self.sess.run(
                    [self.detection_boxes, self.detection_scores,
                     self.detection_classes, self.num_detections],
                    feed_dict={self.image_tensor: image_np_expanded})

            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes).astype(np.int32)

            min_score_thresh = .50
            for i in range(boxes.shape[0]):
                if scores is None or scores[i] > min_score_thresh:

                    class_name = self.category_index[classes[i]]['name']
                    # class_id = self.category_index[classes[i]]['id']  # if needed

                    print('{}'.format(class_name))

                    # Traffic light thing
                    self.current_light = TrafficLight.UNKNOWN

                    if class_name == 'Red':
                        self.current_light = 2
                    elif class_name == 'Green':
                        self.current_light = 1
                    elif class_name == 'Yellow':
                        self.current_light = 3
                    #print(self.current_light)

                    fx = 1345.200806
                    fy = 1353.838257
                    perceived_width_x = (boxes[i][3] - boxes[i][1]) * 800
                    perceived_width_y = (boxes[i][2] - boxes[i][0]) * 600

                    perceived_depth_x = ((1 * fx) / perceived_width_x)
                    perceived_depth_y = ((3 * fy) / perceived_width_y)

                    estimated_distance = round((perceived_depth_x + perceived_depth_y) / 2)

            # # Visualization of the results of a detection.
            # visualize_boxes_and_labels_on_image_array(
            #     image, boxes, classes, scores,
            #     self.category_index,
            #     use_normalized_coordinates=True,
            #     line_thickness=8)

        # For visualization topic output
        #self.image_np_deep = image

        # For timing single frame detection
        #end = time.time()
        #rospy.logwarn("Frame detection time: %f" % (end - start))
        
        return self.current_light
