from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles

import pymysql

app = FastAPI()

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
    try:
        with conn.cursor() as cursor:
            cursor.execute(f"SELECT * FROM ITEM WHERE SECTION_ID = {goal} AND POSITION_NUM = 1;")
            result = cursor.fetchone()
            if result:
                return {
                    "message": "Goal found", 
                    "X": result['X'], 
                    "Y": result['Y'], 
                    "Z": result['Z'], 
                    "W": result['W']
                }
            else:
                return {"message": "Goal not found"}
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
