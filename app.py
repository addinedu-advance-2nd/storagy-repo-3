from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles

import pymysql
import time

import rclpy
from std_msgs.msg import String

from goal_publisher import GoalPublisher

app = FastAPI()

# ROS 초기화
rclpy.init()

# Jinja2 템플릿 설정
templates = Jinja2Templates(directory="templates")

# 정적 파일 설정
app.mount("/static", StaticFiles(directory="static"), name="static")

# index.html 로드
@app.get("/", response_class=HTMLResponse)
async def read_root(request: Request):
    return templates.TemplateResponse("button_index.html", {"request": request})

@app.get("/goal_position/{goal}")
async def get_goal_position(goal: str):
    # Goal Position 조회
    conn = get_db_connection()

    response = {}

    try:
        with conn.cursor() as cursor:
            cursor.execute(f"SELECT * FROM ITEM WHERE SECTION_ID = {goal} AND POSITION_NUM = 1;")
            result = cursor.fetchone()

            if result:
                response = {
                    "message": "Goal found", 
                    "X": result['X'], 
                    "Y": result['Y'], 
                    "Z": result['Z'], 
                    "W": result['W']
                }
            else:
                response = {"message": "Goal not found"}

            print(response)
    finally:
        conn.close()

    # 로봇 주행
    goal_publisher = GoalPublisher()
    time.sleep(3)

    goal_publisher.publish_goal(response['X'], response['Y'])

    rclpy.spin_once(goal_publisher, timeout_sec=2)
    goal_publisher.destroy_node()

    return response

# DB 연결 
def get_db_connection():
    conn = pymysql.connect(
        host='localhost',
        user='root',
        password='12345678',
        database='DAISSOTORAGY',
        cursorclass=pymysql.cursors.DictCursor
    )
    return conn

# FastAPI 서버 실행
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
