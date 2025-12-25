#!/usr/bin/env python3
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

import argparse
import base64
import sys
import threading
import time

import numpy as np
import requests
import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

try:
    from cv_bridge import CvBridge
    import cv2
except Exception:
    CvBridge = None
    cv2 = None


class OdinVLMTerminal:
    def __init__(self, topic, msg_type, server, model, instruction, interval):
        self.topic = topic
        self.msg_type = msg_type
        self.server = server.rstrip('/')
        self.model = model
        self.instruction = instruction
        self.interval = float(interval)

        self._lock = threading.Lock()
        self._latest_data_url = None
        self._latest_cv_img = None
        self._bridge = CvBridge() if CvBridge is not None else None
        # Publisher for annotated image (CompressedImage and Image)
        self._annot_pub = rospy.Publisher('~annotated', CompressedImage, queue_size=1)
        self._annot_pub_raw = rospy.Publisher('~annotated_image', Image, queue_size=1)

        if self.msg_type == 'compressed':
            rospy.loginfo("Subscribing to CompressedImage: %s", self.topic)
            self._sub = rospy.Subscriber(self.topic, CompressedImage, self._on_compressed, queue_size=1)
        else:
            if self._bridge is None or cv2 is None:
                rospy.logerr("cv_bridge and OpenCV are required for raw Image but not available. Install ros-noetic-cv-bridge and python3-opencv.")
                sys.exit(1)
            rospy.loginfo("Subscribing to raw Image: %s", self.topic)
            self._sub = rospy.Subscriber(self.topic, Image, self._on_raw, queue_size=1)

        # Prompt-triggered processing subscriber
        self._prompt_sub = rospy.Subscriber('~prompt', String, self._on_prompt, queue_size=10)

    def _on_compressed(self, msg: CompressedImage):
        try:
            fmt = 'jpeg'
            # msg.format may contain 'jpeg' or 'png'; try to parse
            if msg.format:
                fmt = 'png' if 'png' in msg.format.lower() else 'jpeg'
            b64 = base64.b64encode(msg.data).decode('ascii')
            data_url = f"data:image/{fmt};base64,{b64}"
            # Also decode to cv2 image for annotation if OpenCV is available
            if cv2 is not None:
                npbuf = np.frombuffer(msg.data, dtype=np.uint8)
                img = cv2.imdecode(npbuf, cv2.IMREAD_COLOR)
            else:
                img = None
            with self._lock:
                self._latest_data_url = data_url
                if img is not None:
                    self._latest_cv_img = img
        except Exception as e:
            rospy.logwarn("Failed handling CompressedImage: %s", e)

    def _on_raw(self, msg: Image):
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, enc = cv2.imencode('.jpg', cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ok:
                return
            b64 = base64.b64encode(enc.tobytes()).decode('ascii')
            data_url = f"data:image/jpeg;base64,{b64}"
            with self._lock:
                self._latest_data_url = data_url
                self._latest_cv_img = cv_img
        except Exception as e:
            rospy.logwarn("Failed handling raw Image: %s", e)

    def _get_latest(self):
        with self._lock:
            return self._latest_data_url

    def _get_latest_cv(self):
        with self._lock:
            return None if self._latest_cv_img is None else self._latest_cv_img.copy()

    def _process_instruction(self, instruction: str):
        img_data_url = self._get_latest()
        if img_data_url is None:
            rospy.logwarn("No frame available yet. Cannot process instruction.")
            return
        try:
            content = self._send_to_vlm(instruction, img_data_url)
            print(content, flush=True)
            img_cv = self._get_latest_cv()
            if img_cv is not None:
                self._publish_annotated_image(img_cv, content)
        except Exception as e:
            print(f"Error sending to VLM: {e}", file=sys.stderr, flush=True)

    def _on_prompt(self, msg: String):
        instr = msg.data if msg is not None else ''
        if not instr:
            rospy.logwarn("Received empty prompt on ~prompt")
            return
        self._process_instruction(instr)

    def _publish_annotated_image(self, img, text: str):
        if cv2 is None:
            return
        try:
            annotated = img.copy()
            font = cv2.FONT_HERSHEY_SIMPLEX
            # Param
            font_scale = 3.0
            thickness = 6
            color = (0, 255, 0)
            display_text = text or ""
            # Prepare wrapping for up to two lines across the bottom
            img_h, img_w = annotated.shape[0], annotated.shape[1]
            margin = 10
            line_spacing = 6
            max_lines = 2
            max_width_px = img_w - 2 * margin

            def split_tokens(s: str):
                # Prefer splitting by spaces; if no spaces (e.g., Chinese), fallback to characters
                if ' ' in s:
                    return s.split(' ')
                return list(s)

            def join_tokens(tokens):
                # If tokens are characters (Chinese), join without spaces; else join with single spaces
                if not tokens:
                    return ''
                all_single_chars = all(len(t) == 1 for t in tokens) and (' ' not in tokens)
                return ('' if all_single_chars else ' ').join(tokens)

            tokens = split_tokens(display_text)
            lines = []
            current = []
            for tok in tokens:
                trial = join_tokens(current + [tok])
                size, base = cv2.getTextSize(trial, font, font_scale, thickness)
                if size[0] <= max_width_px or not current:
                    current.append(tok)
                else:
                    # Commit current line and start new
                    lines.append(join_tokens(current))
                    current = [tok]
                if len(lines) >= max_lines:
                    break
            if len(lines) < max_lines and current:
                lines.append(join_tokens(current))

            # If overflow beyond two lines, add ellipsis to the last line to fit
            if len(lines) > max_lines:
                lines = lines[:max_lines]
            # Try to append '...' to indicate truncation if there are remaining tokens
            used_token_count = 0
            # Rough count: rebuild used tokens length by measuring lines; this is conservative
            # If tokens were characters, this sums; if words, approximate via split
            if ' ' in display_text:
                used_token_count = sum(len(l.split(' ')) for l in lines)
            else:
                used_token_count = sum(len(l) for l in lines)
            total_token_count = len(tokens)
            if total_token_count > used_token_count:
                last = lines[-1] if lines else ''
                suffix = '...'
                while True:
                    candidate = (last + suffix) if last else suffix
                    size, _ = cv2.getTextSize(candidate, font, font_scale, thickness)
                    if size[0] <= max_width_px or not last:
                        lines[-1] = candidate
                        break
                    # Trim last line by one token/char
                    if ' ' in display_text:
                        parts = last.split(' ')
                        if len(parts) <= 1:
                            lines[-1] = suffix if cv2.getTextSize(suffix, font, font_scale, thickness)[0] <= max_width_px else ''
                            break
                        last = ' '.join(parts[:-1])
                    else:
                        if len(last) <= 1:
                            lines[-1] = suffix if cv2.getTextSize(suffix, font, font_scale, thickness)[0] <= max_width_px else ''
                            break
                        last = last[:-1]

            # Draw from bottom up
            y = img_h - margin
            for i in range(len(lines)-1, -1, -1):
                line = lines[i]
                if not line:
                    continue
                (w, h), baseline = cv2.getTextSize(line, font, font_scale, thickness)
                y = y - baseline
                cv2.putText(
                    annotated,
                    line,
                    (margin, y),
                    font,
                    font_scale,
                    color,
                    thickness,
                    cv2.LINE_AA,
                )
                y = y - h - line_spacing
            ok, buf = cv2.imencode('.jpg', annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                annotated_msg = CompressedImage()
                annotated_msg.header.stamp = rospy.Time.now()
                annotated_msg.format = 'jpeg'
                annotated_msg.data = buf.tobytes()
                self._annot_pub.publish(annotated_msg)

            if self._bridge is not None:
                try:
                    annotated_raw = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                    annotated_raw.header.stamp = rospy.Time.now()
                    self._annot_pub_raw.publish(annotated_raw)
                except Exception as e:
                    rospy.logwarn("Failed to convert/publish raw annotated image: %s", e)
        except Exception as e:
            rospy.logwarn("Failed to publish annotated image: %s", e)

    def run(self):
        rospy.loginfo("Waiting for first frame on %s (%s)...", self.topic, self.msg_type)
        # Wait for first frame
        while not rospy.is_shutdown():
            if self._get_latest() is not None:
                break
            time.sleep(0.05)
        if rospy.is_shutdown():
            return
        if self.interval <= 0:
            rospy.loginfo("Manual mode: type a prompt in this terminal and press Enter. Ctrl+C to exit.")
            while not rospy.is_shutdown():
                try:
                    prompt = input("prompt> ").strip()
                except (EOFError, KeyboardInterrupt):
                    break
                if not prompt:
                    continue
                self._process_instruction(prompt)
            return

        rospy.loginfo("Auto mode: starting loop with interval=%.3fs", self.interval)
        rate = rospy.Rate(1.0 / self.interval if self.interval > 0 else 10)
        while not rospy.is_shutdown():
            self._process_instruction(self.instruction)
            rate.sleep()

    def _send_to_vlm(self, instruction, image_base64_url):
        # must set stream to llama where you open
        url = f"{self.server}/v1/chat/completions"
        payload = {
            "model": self.model,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": instruction},
                        {"type": "image_url", "image_url": {"url": image_base64_url}},
                    ],
                }
            ],
            # Param
            "max_tokens": 300,
            "temperature": 0.3,
            "stream": False,
        }
        resp = requests.post(url, json=payload, timeout=120)
        if not resp.ok:
            raise RuntimeError(f"Server error {resp.status_code}: {resp.text[:200]}")
        data = resp.json()
        content = (
            data.get("choices", [{}])[0]
            .get("message", {})
            .get("content")
        )
        return content or str(data)


def infer_msg_type(topic: str) -> str:
    return 'compressed' if 'compressed' in topic else 'raw'


def main():
    parser = argparse.ArgumentParser(description="Subscribe to ODIN image topic and query local llama-server, printing responses to terminal.")
    parser.add_argument('--topic', default='/odin1/image/compressed', help='ROS image topic (CompressedImage or Image)')
    parser.add_argument('--msg', choices=['compressed', 'raw'], default=None, help='Message type override')
    parser.add_argument('--server', default='http://localhost:8080', help='llama-server base URL')
    # save model to 
    parser.add_argument('--model', default='SmolVLM-500M-Instruct', help='Model name')
    parser.add_argument('--instruction', default='describe what you see and in detailed', help='Instruction text')
    parser.add_argument('--interval', type=float, default=0.0, help='Seconds between inferences')

    # Use rospy.myargv() to strip ROS-specific arguments like __name, __log, remaps
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('odin_vlm_terminal', anonymous=True)

    # ROS param overrides (take precedence if provided)
    topic = rospy.get_param('~topic', args.topic)
    server = rospy.get_param('~server', args.server)
    model = rospy.get_param('~model', args.model)
    instruction = rospy.get_param('~instruction', args.instruction)
    interval = rospy.get_param('~interval', args.interval)
    msg_param = rospy.get_param('~msg', args.msg)
    msg_type = msg_param or infer_msg_type(topic)

    node = OdinVLMTerminal(
        topic=topic,
        msg_type=msg_type,
        server=server,
        model=model,
        instruction=instruction,
        interval=interval,
    )

    try:
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
