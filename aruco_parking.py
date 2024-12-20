import os
import sys
import cv2
import numpy as np
import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class ArUcoMarkerController(Node):
    def __init__(self):
        super().__init__('aruco_marker_controller')

        # ArUco 마커 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.marker_size = 0.05  # 50mm = 0.05m

        # 캘리브레이션 데이터 로드
        calib_data_path = '/home/storagy/Desktop/jyh/calib_data.npz'
        calib_data = np.load(calib_data_path)
        self.cmtx = calib_data['mtx']  # 3x3 카메라 행렬
        self.dist = calib_data['dist']  # 왜곡 계수

        # CvBridge 초기화
        self.bridge = CvBridge()

        # ROS 이미지 구독 설정
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # ROS 카메라 토픽 이름
            self.image_callback,
            1
        )

        # ROS 제어 명령 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 타이머 설정
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.rgb_image = np.zeros((480, 640, 3), dtype=np.uint8)  # 기본 이미지

        # 제어 목표값
        self.target_tvec_x = 0.00
        self.target_tvec_z = 0.20
        self.target_rvec_y = 0.00

        # 제어 게인 설정
        self.Kp_x = 0.1  # tvec_x 제어 게인
        self.Kp_z = 0.1  # tvec_z 제어 게인
        self.Kp_y = 0.1  # rvec_y 제어 게인

        # 이전 angular.z 값 저장
        self.prev_angular_z = 0.0

    def image_callback(self, msg):
        # ROS Image 메시지를 OpenCV 이미지로 변환
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def timer_callback(self):
        # ArUco 마커 검출
        corners, ids, rejected = cv2.aruco.detectMarkers(self.rgb_image, self.aruco_dict, parameters=self.parameters)

        twist = Twist()  # Twist 메시지 초기화

        if ids is not None:  # 마커가 감지된 경우
            frame = cv2.aruco.drawDetectedMarkers(self.rgb_image.copy(), corners, ids)

            for i, marker_id in enumerate(ids):
                # Pose Estimate 계산
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.marker_size, self.cmtx, self.dist)

                # 좌표축 그리기
                frame = cv2.drawFrameAxes(frame, self.cmtx, self.dist, rvec, tvec, 0.05)

                # 출력 형식 조정
                rvec = np.rad2deg(np.squeeze(rvec))  # 회전 벡터를 각도로 변환
                tvec = np.squeeze(tvec)

                # 현재 위치와 목표 위치의 차이 계산
                diff_x = tvec[0] - self.target_tvec_x
                diff_z = tvec[2] - self.target_tvec_z
                diff_yaw = rvec[1] - self.target_rvec_y

                # 허용 오차 설정
                tolerance_x = 0.05  # 1cm 허용 오차
                tolerance_z = 0.04  # 1cm 허용 오차
                tolerance_yaw = 3.0  # 2도 허용 오차

                if abs(diff_x) < tolerance_x and abs(diff_z) < tolerance_z and abs(diff_yaw) < tolerance_yaw:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                    # 목표 위치 도달 메시지와 값 출력
                    self.get_logger().info(
                        f"목표 위치에 도달: tvec_x={tvec[0]:.2f}, tvec_z={tvec[2]:.2f}, rvec_y={rvec[1]:.2f}"
                    )

                    # 목표 위치 값을 프레임에 표시
                    text_goal = f"Goal: tvec_x={tvec[0]:.2f}, tvec_z={tvec[2]:.2f}, rvec_y={rvec[1]:.2f}"
                    cv2.putText(frame, text_goal, (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                else:
                    # P 제어 계산
                    linear_x = self.Kp_z * diff_z
                    angular_z = self.Kp_x * diff_x + self.Kp_y * diff_yaw

                    # 제어 값 제한 (회전 속도를 더 부드럽게 제한)
                    twist.linear.x = max(min(linear_x, 0.1), -0.1)
                    twist.angular.z = max(min(angular_z, 0.2), -0.2)  # 회전 속도 제한

                    # 이전 angular.z 값을 업데이트
                    self.prev_angular_z = twist.angular.z

                    # 상태 출력
                    self.get_logger().info(
                        f"tvec_x={tvec[0]:.2f}, tvec_z={tvec[2]:.2f}, rvec_y={rvec[1]:.2f}, "
                        f"linear_x={linear_x:.2f}, angular_z={angular_z:.2f}"
                    )
        else:
            # 예외 처리: 마커가 감지되지 않으면 반대 방향으로 회전
            self.get_logger().warn("ArUco 마커가 감지되지 않았습니다. 반대 방향으로 회전합니다.")
            
            # 이전 회전 방향 반전 및 제한
            if self.prev_angular_z == 0.0:  # 초기값 처리
                twist.angular.z = 0.1  # 기본 회전 속도 (느리게 설정)
            else:
                twist.angular.z = -self.prev_angular_z * 0.8  # 이전 회전 방향의 반대값 (속도를 80%로 감소)
            twist.linear.x = 0.0

        # Twist 메시지 퍼블리시
        self.cmd_vel_pub.publish(twist)

        # 결과 이미지 출력
        cv2.imshow("ArUco Marker Control", frame if ids is not None else self.rgb_image)

        # ESC 키를 눌러 종료
        if cv2.waitKey(1) == 27:  # ESC 키
            self.get_logger().info("프로그램을 종료합니다.")
            sys.exit(0)





    def __del__(self):
        cv2.destroyAllWindows()


def main(args=None):
    rp.init(args=args)
    node = ArUcoMarkerController()

    node.get_logger().info("ArUco Marker Controller Node started.")
    try:
        rp.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("프로그램이 종료되었습니다.")
    finally:
        node.destroy_node()
        rp.shutdown()


if __name__ == '__main__':
    main()