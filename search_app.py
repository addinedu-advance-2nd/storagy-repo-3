from fastapi import FastAPI, Request, Query,  HTTPException
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles

import pymysql
from decimal import Decimal

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

@app.get("/search")
async def search(query: str = Query(...)):
    conn = get_db_connection()

    try:
        with conn.cursor() as cursor:
            cursor.execute(f'''
                                SELECT 
                                    (SELECT NAME FROM SECTION WHERE ID = SECTION_ID) AS SECTION_NAME,
                                    NAME AS ITEM_NAME,
                                    CONCAT(SECTION_ID, '-', POSITION_NUM) AS POSITION
                                  FROM ITEM 
                                 WHERE NAME LIKE "%{query}%"
                            ''')
            result = cursor.fetchone()

            if result:
                response = {
                    "section_name": result['SECTION_NAME'],
                    "item_name": result['ITEM_NAME'],
                    "position": result['POSITION'],
                }
                return JSONResponse(content=response)  # 200 상태 코드로 자동 반환
            else:
                return JSONResponse(content={})  # 결과가 없을 경우 빈 응답 반환

    finally:
        conn.close()


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
