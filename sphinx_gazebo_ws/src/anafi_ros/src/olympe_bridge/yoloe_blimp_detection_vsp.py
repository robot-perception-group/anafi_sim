#!/usr/bin/env python3
"""
ROS1 node: subscribe to sensor_msgs/Image, run YOLOE with VISUAL prompts,
draw boxes (and optional masks), and publish annotated image.

Example:
  rosrun your_pkg yoloe_blimp_detection.py \
    --image-topic /camera/image_raw \
    --output-topic /yoloe/annotated \
    --checkpoint ../yoloe/weights/yoloe-v8s-seg.pt \
    --refer-image prompt_images/ref_image.png \
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
from ultralytics.models.yolo.yoloe import YOLOEVPSegPredictor
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--image-topic",
        type=str,
        default="/anafi_camera/image_raw_gazebo_timestamp",
        help="Input ROS image topic (sensor_msgs/Image)"
    )
    parser.add_argument(
        "--output-topic",
        type=str,
        default="/yoloe/annotated",
        help="Output ROS image topic"
    )
    parser.add_argument(
        "--checkpoint",
        type=str,
        default="../yoloe/weights/yoloe-v8s-seg.pt",
        help="Path or ID of the model checkpoint"
    )
    parser.add_argument(
        "--refer-image",
        nargs="+",
        required=True,
        help="Path(s) to reference image(s) used for visual prompting"
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda:0",
        help="Device to run inference on (e.g. cpu, cuda:0)"
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.25,
        help="Confidence threshold"
    )
    parser.add_argument(
        "--iou",
        type=float,
        default=0.7,
        help="IoU threshold"
    )
    parser.add_argument(
        "--encoding",
        type=str,
        default="bgr8",
        help="Published image encoding (bgr8 or rgb8)"
    )
    parser.add_argument(
        "--node-name",
        type=str,
        default="yoloe_visual_prompt_inference",
        help="ROS node name"
    )
    parser.add_argument(
        "--queue-size",
        type=int,
        default=1,
        help="Subscriber queue size"
    )
    parser.add_argument(
        "--no-masks",
        action="store_true",
        help="Disable mask drawing"
    )

    return parser.parse_args()


class YoloeRosNode:
    def __init__(self, args):
        self.args = args
        self.bridge = CvBridge()

        # --- Load model ---
        self.model = YOLOE(args.checkpoint)
        self.model.to(args.device)

        # ------------------------------------------------------------------
        # Visual prompt setup
        #
        # These are the same style of prompts used in vsp_vid.py:
        #   visual_prompts = {"bboxes": ..., "cls": ...}
        #
        # IMPORTANT:
        # These boxes must correspond to the object(s) in your reference image.
        # Replace them with the coordinates that match your own reference image.
        # ------------------------------------------------------------------
        self.refer_image = "/home/pgoldschmid/src/test_apriltags/src/anafi_sim/sphinx_gazebo_ws/src/anafi_ros/src/olympe_bridge/yoloe_prompt_images/ref_image_sim.png"
        self.visual_prompts = {
            "bboxes": np.array([
                
                    [318.44018363964045, 139.08406450201755, 399.321538972427, 218.3638088381146],
                    [1272.165333808739, 179.86246670733163, 1387.6228227595561, 446.87038284972374],
                    [1104.9747253293435, 564.7351793464153, 1273.0892134437818, 951.9782071410523],
                    [320.4497639683217, 960.7376295180698, 493.997562460441, 1081.5353372831014],
                    [1451.671345124986, 1101.907878802666, 1642.5781938755222, 1207.784378253394],
                    [51.17029302753963, 1510.1145000105623, 95.344595565374, 1567.8182014806487],
                    [1553.9865717933842, 1620.6142850745987, 1638.0445093211142, 1699.7992986876768],
                    [726.0026028632908, 580.3012954688204, 735.7526028632908, 601.6762954688204]

            ], dtype=np.float32),
            "cls": np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.int32),
        }
        self.mask_annotator = sv.MaskAnnotator(
            color_lookup=sv.ColorLookup.INDEX,
            opacity=0.4
        )
        self.box_annotator = None
        self.label_annotator = None

        self.pub = rospy.Publisher(args.output_topic, Image, queue_size=1)

        self._lock = threading.Lock()
        self.frame_count = 0
        self.last_msg = None
        self.detections = sv.Detections.empty()

        self.sub = rospy.Subscriber(
            args.image_topic,
            Image,
            self.image_cb,
            queue_size=args.queue_size,
            buff_size=2**24
        )

        #Initialize empty output image
        h=300
        w=300
        self.out_cv = np.zeros((h, w, 3), dtype=np.uint8)



        rospy.loginfo(
            "YOLOE visual-prompt ROS node ready. Subscribing to %s, publishing to %s",
            args.image_topic,
            args.output_topic
        )

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

    def image_cb(self, msg):
        if not self._lock.acquire(blocking=False):
            return
        

        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            h, w = frame_bgr.shape[:2]
            self._init_annotators_if_needed(w, h)

            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

            # Visual-prompt inference
            results = self.model.predict(
                source=frame_rgb,
                refer_image=self.refer_image,
                visual_prompts=self.visual_prompts,
                predictor=YOLOEVPSegPredictor,
                verbose=False,
                conf=self.args.conf,
                iou=self.args.iou,
            )

            result = results[0]
            detections = sv.Detections.from_ultralytics(result)
            self.detections = detections

            labels = []
            if len(detections) > 0:
                if "class_name" in detections.data:
                    labels = [
                        f"{class_name} {confidence:.2f}"
                        for class_name, confidence in zip(
                            detections["class_name"], detections.confidence
                        )
                    ]
                else:
                    labels = [
                        f"blimp {confidence:.2f}"
                        for confidence in detections.confidence
                    ]

            annotated_rgb = frame_rgb.copy()

            if not self.args.no_masks:
                try:
                    annotated_rgb = self.mask_annotator.annotate(
                        scene=annotated_rgb,
                        detections=detections
                    )
                except Exception:
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

            if self.args.encoding.lower() == "rgb8":
                out_cv = annotated_rgb
                out_enc = "rgb8"
            else:
                out_cv = cv2.cvtColor(annotated_rgb, cv2.COLOR_RGB2BGR)
                out_enc = "bgr8"
            
            self.out_cv = out_cv

            out_msg = self.bridge.cv2_to_imgmsg(out_cv, encoding=out_enc)
            out_msg.header = msg.header
            self.pub.publish(out_msg)

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