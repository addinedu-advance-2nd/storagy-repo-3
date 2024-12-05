from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles
import rclpy
from std_msgs.msg import String

app = FastAPI()

# Jinja2 템플릿 설정
templates = Jinja2Templates(directory="templates")

# 정적 파일 설정
app.mount("/static", StaticFiles(directory="static"), name="static")

# ROS 초기화
rclpy.init()
node = rclpy.create_node('message_publisher')
publisher = node.create_publisher(String, 'user_message', 10)

@app.get("/", response_class=HTMLResponse)
async def read_root(request: Request):
    return templates.TemplateResponse("button_index.html", {"request": request})

@app.post("/publish")
async def publish(message: dict):
    msg = String()
    msg.data = message['message']
    publisher.publish(msg)
    node.get_logger().info(f"메시지 발행: {msg.data}")
    return {"status": "메시지가 발행되었습니다."}

# FastAPI 서버 실행
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
