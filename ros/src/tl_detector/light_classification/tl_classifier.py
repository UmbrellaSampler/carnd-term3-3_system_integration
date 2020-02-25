from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import datetime


class TLClassifier(object):
    def __init__(self, is_site):

        self.graph = tf.Graph()

        # Threshold about how confident we need to be to trust the classification
        self.threshold = .5

        # Choose the right model graph: simulation or site
        if is_site:
            model_graph = r'light_classification/trained_model/ssd_inception_v2_coco_2017_11_17/site/frozen_inference_graph.pb'
        else:
            model_graph = r'light_classification/trained_model/ssd_inception_v2_coco_2017_11_17/sim/frozen_inference_graph.pb'

        with self.graph.as_default():
            graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_graph, 'rb') as file:
                graph_def.ParseFromString(file.read())
                tf.import_graph_def(graph_def, name='')

            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            self.scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.graph.get_tensor_by_name(
                'num_detections:0')

        self.sess = tf.Session(graph=self.graph)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        with self.graph.as_default():
            img_conv = np.expand_dims(image, axis=0)
            start = datetime.datetime.now()
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict={self.image_tensor: img_conv})
            end = datetime.datetime.now()
            c = end - start
            print(c.total_seconds())

        score = np.squeeze(scores)[0]
        clazz = np.squeeze(classes).astype(np.int32)[0]

        print('SCORES: ', score)
        print('CLASSES: ', clazz)

        if score> self.threshold:
            if clazz == 1:
                print('green')
                return TrafficLight.GREEN
            elif clazz == 2:
                print('red')
                return TrafficLight.RED
            elif clazz == 3:
                print('yellow')
                return TrafficLight.YELLOW

        return TrafficLight.UNKNOWN
