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
    return templates.TemplateResponse("search_index.html", {"request": request})

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
