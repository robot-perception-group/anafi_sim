#!/usr/bin/env python3
"""
ROS1 node: subscribe to sensor_msgs/Image, run YOLOE, draw boxes (and optional masks),
publish annotated image to another topic.

Deps:
  - rospy
  - sensor_msgs
  - cv_bridge
  - opencv-python
  - supervision
  - ultralytics (YOLOE)

Example:
  rosrun your_pkg yoloe_ros_node.py \
    --image-topic /camera/image_raw \
    --output-topic /yoloe/annotated \
    --checkpoint yoloe-v8l-seg.pt \
    --names person \
    --device cuda:0
"""

import argparse
import threading

import cv2
import rospy
import supervision as sv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLOE
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser()
    # parser.add_argument("--image-topic", type=str, default="/anafi_camera/image_raw_gazebo_timestamp",
    #                     help="Input ROS image topic (sensor_msgs/Image)")
    parser.add_argument("--output-topic", type=str, default="/yoloe/annotated",
                        help="Output ROS image topic (sensor_msgs/Image)")
    parser.add_argument("--checkpoint", type=str, default="../yoloe/weights/yoloe-v8s-seg.pt",
                        help="Path or ID of the model checkpoint")
    parser.add_argument("--names", nargs="+", default=["blimp drone","drone spherical with attachments"],
                        help="List of open-vocab class names to set for the model")
    parser.add_argument("--device", type=str, default="cuda",
                        help="Device to run inference on (e.g. cpu, cuda:0)")
    parser.add_argument("--conf", type=float, default=0.25,
                        help="Confidence threshold")
    parser.add_argument("--iou", type=float, default=0.7,
                        help="IoU threshold")

    parser.add_argument("--encoding", type=str, default="rgb8",
                        help="Output image encoding for published message (bgr8 or rgb8 are common)")
    parser.add_argument("--node-name", type=str, default="yoloe_inference",
                        help="ROS node name")
    parser.add_argument("--queue-size", type=int, default=1,
                        help="Subscriber queue size (1 drops frames if inference is slow)")
    return parser.parse_args()



class YoloeRosNode:
    def __init__(self, args):
        self.args = args
        self.args.no_masks = True
        self.bridge = CvBridge()

        # --- Load model ---
        self.model = YOLOE(args.checkpoint)
        self.model.to("cuda:0")

        # Set open-vocab classes once
        self.model.set_classes(args.names, self.model.get_text_pe(args.names))

        # Annotators (thickness/text_scale depend on resolution; set lazily on first frame)
        self.mask_annotator = sv.MaskAnnotator(color_lookup=sv.ColorLookup.INDEX, opacity=0.4)
        self.box_annotator = None
        self.label_annotator = None

        # self.pub = rospy.Publisher(args.output_topic, Image, queue_size=1)

        # If inference is slow, this lock prevents re-entrancy if callbacks overlap
        self._lock = threading.Lock()

        self._image = np.zeros((3,3,3))
        self.out_cv = self._image

        # Use a large buff_size for big images (compressed drivers can still publish raw images)
        # print("args image topic",args.image_topic)
        # self.sub = rospy.Subscriber(
        #     args.image_topic,
        #     Image,
        #     self.image_cb
        #                     )

        self.frame_count = 0
        rospy.loginfo("YOLOE ROS node ready. Subscribing to %s, publishing to %s",
                      args.image_topic, args.output_topic)
        

        self.detections = []

    def _init_annotators_if_needed(self, width, height):
        if self.box_annotator is not None and self.label_annotator is not None:
            return

        resolution_wh = (width, height)
        thickness = sv.calculate_optimal_line_thickness(resolution_wh=resolution_wh)
        text_scale = sv.calculate_optimal_text_scale(resolution_wh=resolution_wh)

        self.box_annotator = sv.BoxAnnotator(
            color_lookup=sv.ColorLookup.INDEX,
            thickness=thickness
        )
        self.label_annotator = sv.LabelAnnotator(
            color_lookup=sv.ColorLookup.INDEX,
            text_scale=text_scale,
            smart_position=True
        )

    @property
    def image(self):
        return self._image

    @image.setter
    def image(self,value):
        old = self._image
        self._image = value
        # print("dae",r,p,y)
        if value != old:
            self.image_cb()
        return 

    def image_cb(self):
        # Drop frames if we?re still processing the previous one
        if not self._lock.acquire(blocking=False):
            return

        try:
            # Convert ROS -> OpenCV BGR
            frame_bgr = self.bridge.imgmsg_to_cv2(self._image, desired_encoding="bgr8")
            # self.out_cv = frame_bgr
            h, w = frame_bgr.shape[:2]
            self._init_annotators_if_needed(w, h)

            # Ultralytics expects RGB
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

            results = self.model.predict(
                frame_rgb,
                verbose=False,
                conf=self.args.conf,
                iou=self.args.iou
            )

            detections = sv.Detections.from_ultralytics(results[0])
            self.detections = detections

            labels = [
                f"{class_name} {confidence:.2f}"
                for class_name, confidence in zip(
                    detections["class_name"], detections.confidence
                )
            ]

            annotated_rgb = frame_rgb.copy()

            if not self.args.no_masks:
                try:
                    annotated_rgb = self.mask_annotator.annotate(
                        scene=annotated_rgb,
                        detections=detections
                    )
                except Exception:
                    # If the model/checkpoint doesn't output masks, skip
                    pass

            annotated_rgb = self.box_annotator.annotate(
                scene=annotated_rgb,
                detections=detections
            )
            annotated_rgb = self.label_annotator.annotate(
                scene=annotated_rgb,
                detections=detections,
                labels=labels
            )

            # Publish annotated image
            out_cv = annotated_rgb
            if self.args.encoding.lower() == "rgb8":
                out_cv = annotated_rgb
                out_enc = "rgb8"

            else:
                out_cv = cv2.cvtColor(annotated_rgb, cv2.COLOR_RGB2BGR)
                out_enc = "bgr8"
            self.out_cv = out_cv

            # self.out_cv = self.bridge.cv2_to_imgmsg(out_cv, encoding="bgr8")
            # # Preserve timing + frame_id for downstream consumers 
            # self.pub.publish(out_msg)

            self.frame_count += 1
            if self.frame_count % 50 == 0:
                rospy.loginfo("Processed %d frames", self.frame_count)

        except Exception as e:
            rospy.logerr("Error in callback: %s", str(e))
        finally:
            self._lock.release()


def main():
    args = parse_args()
    rospy.init_node(args.node_name, anonymous=False)
    _node = YoloeRosNode(args)
    rospy.spin()


if __name__ == "__main__":
    main()
