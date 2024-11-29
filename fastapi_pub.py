from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
import rclpy
from std_msgs.msg import String
import threading

app = FastAPI()

# Jinja2 템플릿 설정
templates = Jinja2Templates(directory="templates")

# ROS 초기화
rclpy.init()
node = rclpy.create_node('message_handler')

# 전역 변수 설정
latest_message = "발행된 메시지가 없습니다."

def message_callback(msg):
    global latest_message
    latest_message = msg.data  # 수신한 메시지를 저장

def create_subscriber():
    # ROS2 구독 설정
    node.create_subscription(String, 'user_message', message_callback, 10)

@app.get("/", response_class=HTMLResponse)
async def read_root(request: Request):
    return templates.TemplateResponse("index.html", {"request": request, "message": latest_message})

@app.post("/publish")
async def publish(message: dict):
    msg = String()
    msg.data = message['message']
    publisher = node.create_publisher(String, 'user_message', 10)
    publisher.publish(msg)
    node.get_logger().info(f"발행된 메시지: {msg.data}")
    return {"status": "메시지가 발행되었습니다."}

@app.get("/latest_message")
async def get_latest_message():
    return {"message": latest_message}

# ROS2 노드 실행을 위한 스레드 설정
def ros_spin():
    rclpy.spin(node)

# FastAPI 서버 실행
if __name__ == "__main__":
    # ROS2 구독 함수 호출
    create_subscriber()

    # ROS2 spin을 위한 스레드 시작
    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.start()

    # FastAPI 서버 실행
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
