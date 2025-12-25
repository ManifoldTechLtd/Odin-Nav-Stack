'''
Copyright 2025 Manifold Tech Ltd.(www.manifoldtech.com.co)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
'''

from enum import Enum
import base64
import sys, os
from openai import OpenAI
import time
import threading
from loguru import logger
import json
import atexit
import traceback

import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import quaternion_from_euler


class CompressionType(Enum):
    NONE = 0
    JPEG = 1
    WEBP = 2


TOPIC_IMAGE = "/odin1/image/compressed"
TOPIC_CMD_STR = "/cmd_str"
TOPIC_GOAL = "/neupan/goal"

IMG_SIZE = (320, 240)
COMPRESS_TYPE = CompressionType.WEBP
JPEG_QUALITY = 70
WEBP_LOSSLESS = False
WEBP_QUALITY = 70

REQUEST_THROTTLE_SEC = 1.0

# PROMPT = """
# You are a robot dog navigation expert analyzing a front-facing camera view.

# Task: Identify obstacle areas in images

# Estimate free space:
# The image is divided into three areas by two blue lines.
# For each segmented area, determine the potential obstacle distance based on factors such as free space and pedestrians, assigning a score between 0 and 10.
# A score of 0 indicates complete obstruction, while a score of 10 indicates no obstruction.

# The input image is captured with a fisheye lens, causing obstacles to appear closer than they actually are in the image.
# Additionally, distortion is more pronounced on the left and right sides of the image.
# Estimates for the left and right edge regions should be made more conservatively.

# You should output in JSON format as follows:
# {"left": <score>,"center": <score>,"right": <score>}
# You don't need any additional output beyond the JSON.
# """

# PROMPT = """
# You are a robot dog navigation expert analyzing a front-facing camera view.

# Task: Follow the person with black clothes while avoiding obstacles.

# The image is divided into three areas by two blue lines.

# If the person is not visible, stop moving.
# If the person is in the left area, output left.
# If the person is in the right area, output right.
# If the person is in the center area, output forward.
# If the person is in the center area and is close, output stop.

# You should output in JSON format as follows:
# {"command": "<command>"}
# You don't need any additional output beyond the JSON.
# """


class VLMClient:
    def __init__(self):
        rospy.init_node("vlm_client", anonymous=True)

        self._img_lock = threading.Lock()
        self._img_event = threading.Event()
        self._latest_img_msg = None
        self._worker_thread = threading.Thread(target=self._image_worker, daemon=True)

        self.bridge = CvBridge()
        self._use_compressed = "compressed" in TOPIC_IMAGE
        if self._use_compressed:
            self.image_sub = rospy.Subscriber(
                TOPIC_IMAGE, CompressedImage, self._compressed_img_cb, queue_size=1
            )
        else:
            self.image_sub = rospy.Subscriber(
                TOPIC_IMAGE, Image, self._image_cb, queue_size=1
            )

        self.goal_pub = rospy.Publisher(TOPIC_GOAL, PoseStamped, queue_size=1)
        self.str_cmd_pub = rospy.Publisher(TOPIC_CMD_STR, String, queue_size=1)
        annotated_topic = f"{TOPIC_IMAGE}/annotated_image"
        annotated_type = CompressedImage if self._use_compressed else Image
        self.annotated_img_pub = rospy.Publisher(
            annotated_topic, annotated_type, queue_size=1
        )
        self.annotated_img_raw_pub = rospy.Publisher(
            f"{annotated_topic}/raw", annotated_type, queue_size=1
        )

        self.client = OpenAI(
            api_key=os.getenv("DASHSCOPE_API_KEY"),
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
        )
        self.total_tokens = 0
        self.completion_tokens = 0
        self.prompt_tokens = 0
        self.prompt_text_tokens = 0
        self.prompt_img_tokens = 0
        self.token_record_file = os.path.expanduser("~/vlm_token_usage.json")
        self.tmp_token_file = "/tmp/vlm_tmp_token.log"
        self._token_lock = threading.Lock()
        self.last_request_time = 0.0

        atexit.register(self._on_exit)

        logger.info(f"Subscribed to image topic: {TOPIC_IMAGE}")
        logger.info(f"Publish annotated images to: {annotated_topic}")

    def _compressed_img_cb(self, msg: CompressedImage):
        with self._img_lock:
            self._latest_img_msg = msg
        self._img_event.set()

    def _image_cb(self, msg: Image):
        with self._img_lock:
            self._latest_img_msg = msg
        self._img_event.set()

    def _image_worker(self):
        while not rospy.is_shutdown():
            if not self._img_event.wait(timeout=0.5):
                continue
            self._img_event.clear()
            with self._img_lock:
                msg = self._latest_img_msg
                self._latest_img_msg = None
            if msg is None:
                continue
            t_now = time.time()
            if t_now - self.last_request_time < REQUEST_THROTTLE_SEC:
                continue
            self.last_request_time = t_now
            try:
                if self._use_compressed:
                    cv_img = self.bridge.compressed_imgmsg_to_cv2(
                        msg, desired_encoding="bgr8"
                    )
                else:
                    cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except CvBridgeError as e:
                logger.error(f"CvBridge conversion failed: {e}")
                logger.info(traceback.format_exc())
                continue

            self.image_vlm_process(cv_img)

    def image_vlm_process(self, img: cv2.Mat) -> str:
        img_resize = cv2.resize(img, IMG_SIZE)
        img = self.image_pre_process(img_resize.copy())

        if COMPRESS_TYPE == CompressionType.JPEG:
            ok, buf = cv2.imencode(
                ".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
            )
        elif COMPRESS_TYPE == CompressionType.WEBP:
            encode_params = []
            if WEBP_LOSSLESS:
                encode_params = [int(cv2.IMWRITE_WEBP_LOSSLESS), 1]
            else:
                encode_params = [int(cv2.IMWRITE_WEBP_QUALITY), int(WEBP_QUALITY)]
            ok, buf = cv2.imencode(".webp", img, encode_params)
        elif COMPRESS_TYPE == CompressionType.NONE:
            ok, buf = cv2.imencode(".png", img)
        else:
            logger.error("Unsupported compression type")
            return ""

        if not ok:
            logger.error("Image compress failed")
            return ""

        img_base64 = base64.b64encode(buf.tobytes()).decode("utf-8")

        if COMPRESS_TYPE == CompressionType.JPEG:
            img_url = f"data:image/jpeg;base64,{img_base64}"
        elif COMPRESS_TYPE == CompressionType.WEBP:
            img_url = f"data:image/webp;base64,{img_base64}"
        else:
            img_url = f"data:image/png;base64,{img_base64}"

        prompt = f"""
You are a robot dog navigation expert analyzing a front-facing camera view.

Task: Follow the person wearing gray clothes.

The image is divided into three areas by two blue lines.

If the no target is visible, output stop.
If the target is in the left area, output left.
If the target is in the right area, output right.
If the target is in the center area, output forward.
If the target is in the center area and is close, output stop. You don't need to get too close, keep a safe distance.

You should also try to avoid obstacles while following the target.
Try to turn to the clear area if there are obstacles in the way.

You should output in JSON format as follows:
{{"command": "<command>"}}
You don't need any additional output beyond the JSON.
"""

        # https://help.aliyun.com/zh/model-studio/getting-started/models
        completion = self.client.chat.completions.create(
            model="qwen3-vl-plus",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "image_url": {"url": img_url},
                        },
                        {"type": "text", "text": prompt},
                    ],
                }
            ],
        )
        response = json.loads(completion.model_dump_json())
        logger.trace(f"VLM full response: {response}")
        logger.trace(f"Total tokens: {response['usage']['total_tokens']}")
        logger.trace(f"Prompt tokens: {response['usage']['prompt_tokens']}")
        logger.trace(f"Completion tokens: {response['usage']['completion_tokens']}")

        try:
            with self._token_lock:
                self.total_tokens += response["usage"]["total_tokens"]
                self.completion_tokens += response["usage"]["completion_tokens"]
                self.prompt_tokens += response["usage"]["prompt_tokens"]
                self.prompt_text_tokens += response["usage"]["prompt_tokens_details"][
                    "text_tokens"
                ]
                self.prompt_img_tokens += response["usage"]["prompt_tokens_details"][
                    "image_tokens"
                ]
        except Exception as e:
            logger.warning(f"Failed to update token counters: {e}")
            logger.info(traceback.format_exc())

        with open(self.tmp_token_file, "a") as f:
            f.write(
                f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Prompt tokens: {self.prompt_tokens}, Completion tokens: {self.completion_tokens}\n"
            )

        result = response["choices"][0]["message"]["content"]
        try:
            result = json.loads(result)
        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse JSON response: {e}")
            result = {}
        logger.info(f"VLM response content: {result}")
        text = result.get("command", "no command")

        self._publish_annotated_image(self.annotated_img_pub, img, text)
        self._publish_annotated_image(self.annotated_img_raw_pub, img_resize, text)

        if text != "no command":
            str_cmd = String()
            str_cmd.data = text
            self.str_cmd_pub.publish(str_cmd)

        return result

    def image_pre_process(self, img: cv2.Mat) -> cv2.Mat:
        # Draw two blue vertical lines at 40% and 60% of image width
        height, width = img.shape[:2]
        line_width = int(width * 0.01)  # 1% of image width

        x1 = int(width * 0.35)
        cv2.line(img, (x1, 0), (x1, height), (255, 0, 0), line_width)
        x2 = int(width * 0.65)
        cv2.line(img, (x2, 0), (x2, height), (255, 0, 0), line_width)
        
        return img

    def _on_exit(self):
        try:
            self._write_token_usage()
        except Exception as e:
            logger.error(f"Error writing token usage on exit: {e}")

    def _write_token_usage(self):
        existing = {
            "total": 0,
            "prompt": {
                "total": 0,
                "text_tokens": 0,
                "image_tokens": 0,
            },
            "completion": 0,
        }
        try:
            if os.path.exists(self.token_record_file):
                with open(self.token_record_file, "r") as f:
                    existing = json.load(f)
        except Exception as e:
            logger.warning(f"Failed to read existing token record file: {e}")

        with self._token_lock:
            new_total = existing["total"] + int(self.total_tokens)
            new_prompt = {
                "total": existing["prompt"]["total"] + int(self.prompt_tokens),
                "text_tokens": existing["prompt"]["text_tokens"]
                + int(self.prompt_text_tokens),
                "image_tokens": existing["prompt"]["image_tokens"]
                + int(self.prompt_img_tokens),
            }
            new_completion = existing["completion"] + int(self.completion_tokens)

        total = new_prompt["total"] + new_completion
        data = {
            "total": total,
            "prompt": {
                "total": new_prompt["total"],
                "text_tokens": new_prompt["text_tokens"],
                "image_tokens": new_prompt["image_tokens"],
            },
            "completion": new_completion,
        }

        logger.info(f"Total token inc:{self.total_tokens}")
        logger.info(f"Prompt token inc:{self.prompt_tokens}")
        logger.info(f"Completion token inc:{self.completion_tokens}")

        try:
            # write atomically via temp file
            tmp_path = f"{self.token_record_file}.tmp"
            with open(tmp_path, "w") as f:
                json.dump(data, f, indent=2)
            os.replace(tmp_path, self.token_record_file)
            logger.info(f"Wrote token usage to {self.token_record_file}: {data}")
        except Exception as e:
            logger.error(f"Failed to write token usage file: {e}")

    def _publish_annotated_image(self, pub:rospy.Publisher, img: cv2.Mat, text: str):
        annotated = img.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.8
        thickness = 2
        color = (0, 255, 0)
        display_text = text or ""
        text_size, baseline = cv2.getTextSize(display_text, font, font_scale, thickness)
        x = 10
        y = annotated.shape[0] - baseline - 10
        cv2.putText(
            annotated,
            display_text,
            (x, y),
            font,
            font_scale,
            color,
            thickness,
            cv2.LINE_AA,
        )

        if self._use_compressed:
            if COMPRESS_TYPE == CompressionType.JPEG:
                ok, buf = cv2.imencode(
                    ".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
                )
                compress_format = "jpeg"
            elif COMPRESS_TYPE == CompressionType.WEBP:
                encode_params = []
                if WEBP_LOSSLESS:
                    encode_params = [int(cv2.IMWRITE_WEBP_LOSSLESS), 1]
                else:
                    encode_params = [int(cv2.IMWRITE_WEBP_QUALITY), int(WEBP_QUALITY)]
                ok, buf = cv2.imencode(".webp", annotated, encode_params)
                compress_format = "webp"
            elif COMPRESS_TYPE == CompressionType.NONE:
                ok, buf = cv2.imencode(".png", annotated)
                compress_format = "png"
            else:
                logger.error("Unsupported compression type")
                return

            if not ok:
                logger.error("Failed to encode annotated image")
                return
            annotated_msg = CompressedImage()
            annotated_msg.header.stamp = rospy.Time.now()
            annotated_msg.format = compress_format
            annotated_msg.data = buf.tobytes()
        else:
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            except CvBridgeError as e:
                logger.warning(f"Failed to convert annotated image: {e}")
                return
            annotated_msg.header.stamp = rospy.Time.now()

        pub.publish(annotated_msg)

    def run(self):
        logger.info("VLM node start.")
        self._worker_thread.start()
        rospy.spin()


if __name__ == "__main__":
    logger.remove()
    logger.add(sys.stderr, level="DEBUG")

    try:
        node = VLMClient()
        node.run()
    except rospy.ROSInterruptException:
        pass
