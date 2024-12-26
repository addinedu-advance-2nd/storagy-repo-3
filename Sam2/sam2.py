import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import threading
import torch
import numpy as np
import os
from sam2.build_sam import build_sam2_camera_predictor
import time
import mediapipe as mp
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# Qos 프로파일 설정
qos_profile = QoSProfile(
    depth=10,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)
import torch
# CUDA 사용 가능 여부 체크
print("CUDA available:", torch.cuda.is_available())
print("CUDA device count:", torch.cuda.device_count())
print("Current device:", torch.cuda.current_device())
print("Device name:", torch.cuda.get_device_name(0))

#sam2 체크포인트, 모델 설정(수정가능)
sam2_checkpoint = "/home/d/sam2_ws/segment-anything-2-real-time/checkpoints/sam2_hiera_base_plus.pt"
model_cfg = "../sam2_configs/sam2_hiera_b+.yaml"
#CUDA 사용
predictor = torch.autocast(device_type="cuda", dtype=torch.bfloat16)(
    lambda: build_sam2_camera_predictor(model_cfg, sam2_checkpoint)
)()
#스토리지 로봇에서 발행하는 
class SubNode(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        #스토리지 로봇에서 발행하는 이미지 수신
        self.subscription = self.create_subscription(
            CompressedImage,
            'compressed_image',
            self.listener_callback_rgb,
            qos_profile
        )
        #객체 인식 상태 정보 발행
        self.publisher = self.create_publisher(Bool, 'mask_detection_status', qos_profile)
        self.bridge = CvBridge()
        self.latest_frame = None
        self.lock = threading.Lock()
        self.if_init = False
        self.bbox = None
        self.frame_count = 0
        self.mask_overlay = None
        #ROS2 navigation2 API
        self.navigator = BasicNavigator()
        #미디어파이프 설정
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.v_gesture_count = 0
        self.thumbsup_gesture_count = 0
        self.required_frames = 9990 
        self.required_frames_V = 60
        cv2.namedWindow("frame")

    #압축이미지 수신 및 압축해제
    def listener_callback_rgb(self, msg):
        try:
            image_np = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            with self.lock:
                self.latest_frame = image_np
        except Exception as e:
            self.get_logger().error(f"Error decoding image: {e}")

    #V제스쳐 및 엄지척 제스쳐 식별
    def detect_V(self, frame):
        results = self.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        pt1 = (160,120)
        pt2 = (480,470)
        cv2.rectangle(frame,pt1,pt2,(0,255,0),2)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style()
                )
                lm = hand_landmarks.landmark
                thumb_tip = lm[self.mp_hands.HandLandmark.THUMB_TIP]
                index_tip = lm[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
                middle_tip = lm[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
                ring_tip = lm[self.mp_hands.HandLandmark.RING_FINGER_TIP]
                pinky_tip = lm[self.mp_hands.HandLandmark.PINKY_TIP]
                #V제스쳐 인식
                if (
                    index_tip.y < thumb_tip.y
                    and middle_tip.y < index_tip.y
                    and ring_tip.y > middle_tip.y
                    and pinky_tip.y > middle_tip.y
                ):
                    self.v_gesture_count += 1
                    self.thumbsup_gesture_count = 0
                #엄지척 제스쳐 인식
                elif (
                    index_tip.y > thumb_tip.y
                    and middle_tip.y > thumb_tip.y
                ):  
                    self.thumbsup_gesture_count += 1
                    self.v_gesture_count = 0
                else:
                    self.v_gesture_count = 0
                    self.thumbsup_gesture_count = 0

                if self.v_gesture_count >= self.required_frames_V:
                    self.v_gesture_count = 0
                    return "V"
                if self.thumbsup_gesture_count >= self.required_frames:
                    self.thumbsup_gesture_count = 0
                    return "THUMBS_UP"

        return None
    #SAM2 모델 적용
    def display_image(self):
        fps_start_time = time.time()

        while rclpy.ok():
            frame = self.latest_frame
            detection_status = False
            with self.lock:
                if self.latest_frame is not None:
                    frame = self.latest_frame.copy()

            if frame is not None:
                height, width = frame.shape[:2]
                self.frame_count += 1
                gesture = self.detect_V(frame)
                #V자 제스쳐가 인식되면 SAM2 예측 시작
                if not self.if_init and gesture == "V":
                    pt1 = (160,120)
                    pt2 = (480,470)
                    self.bbox2 = [pt1, pt2]
                    bbox_np2 = np.array([pt1, pt2], dtype=np.float32)
                    predictor.load_first_frame(frame)
                    ann_frame_idx = 0
                    ann_obj_id = 1
                    predictor.add_new_prompt(frame_idx=ann_frame_idx, obj_id=ann_obj_id, bbox=bbox_np2)
                    self.if_init = True
                #엄지척 제스쳐가 인식되면 예측 종료
                elif self.if_init and gesture == "THUMBS_UP":
                    self.if_init = False
                    self.mask_overlay = None
                    detection_status = False

                
                elif self.if_init and self.frame_count % 1 == 0:
                    out_obj_ids, out_mask_logits = predictor.track(frame)
                    all_mask = np.zeros((height, width), dtype=np.uint8)
                    for i in range(len(out_obj_ids)):
                        out_mask = (out_mask_logits[i] > 0.0).permute(1, 2, 0).cpu().numpy().astype(np.uint8) * 255
                        out_mask = out_mask[:, :, 0]
                        if np.any(out_mask):
                            detection_status = True
                        else:
                            detection_status = False

                        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                        out_mask = cv2.morphologyEx(out_mask, cv2.MORPH_CLOSE, kernel)
                        out_mask = cv2.morphologyEx(out_mask, cv2.MORPH_OPEN, kernel)
                        all_mask = cv2.bitwise_or(all_mask, out_mask) 

                    self.mask_overlay = all_mask
                #객체 인식여부 Bool 형태로 발행(인식된 객체가 있으면 True)
                self.publisher.publish(Bool(data=detection_status))     

                if self.mask_overlay is not None:
                    all_mask_rgb = np.zeros_like(frame)
                    all_mask_rgb[:, :, 0] = self.mask_overlay
                    frame = cv2.addWeighted(frame, 1, all_mask_rgb, 0.5, 0)   



                cv2.imshow("frame", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                # FPS 출력
                fps = 1.0 / (time.time() - fps_start_time)
                print(f'FPS: {fps:.2f} Hz')
                fps_start_time = time.time()

        cv2.destroyAllWindows()

    def destroy_node(self):
        self.hands.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SubNode()
    display_thread = threading.Thread(target=node.display_image, daemon=True)
    mediapipe_thread = threading.Thread(target=node.detect_V, daemon= True)
    display_thread.start()
    mediapipe_thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()