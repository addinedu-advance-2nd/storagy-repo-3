from fastapi import FastAPI, Request, Query,  HTTPException
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles

import pymysql
from decimal import Decimal

import rclpy
from rclpy.executors import MultiThreadedExecutor
import queue
from base_BasicNavigator import DetectionSubscriber, NavigationNode
from navigation_client import NavigationClient

import time

app = FastAPI()

# Jinja2 템플릿 설정
templates = Jinja2Templates(directory="templates")

# 정적 파일 설정
app.mount("/static", StaticFiles(directory="static"), name="static")

# index.html 로드
@app.get("/", response_class=HTMLResponse)
async def read_root(request: Request):
    return templates.TemplateResponse("search_index.html", {"request": request})

@app.post("/register-user")
async def start_guide():  
    try:
        # 사용자 등록
        time.sleep(3)

        raise HTTPException(status_code=200)
    except (ValueError, TypeError) as e:
        raise HTTPException(status_code=400, detail="Failed to register the user.")

@app.get("/search")
async def search(query: str = Query(...)):
    conn = get_db_connection()

    try:
        with conn.cursor() as cursor:
            cursor.execute(f'''
                                SELECT 
                                    (SELECT NAME FROM SECTION WHERE ID = SECTION_ID) AS SECTION_NAME,
                                    NAME AS ITEM_NAME,
                                    CONCAT(SECTION_ID, '-', ITEM_NUM) AS ITEM_NUM,
                                    (SELECT X FROM GOAL_POSITION WHERE ID = GOAL_POSITION_ID) AS X,
                                    (SELECT Y FROM GOAL_POSITION WHERE ID = GOAL_POSITION_ID) AS Y,
                                    (SELECT Z FROM GOAL_POSITION WHERE ID = GOAL_POSITION_ID) AS Z,
                                    (SELECT W FROM GOAL_POSITION WHERE ID = GOAL_POSITION_ID) AS W
                                FROM ITEM 
                                WHERE NAME LIKE "%{query}%"
                            ''')
            result = cursor.fetchone()

            if result:
                response = {
                    "section_name": result['SECTION_NAME'],
                    "item_name": result['ITEM_NAME'],
                    "item_num": result['ITEM_NUM'],
                    "x": float(result['X']) if isinstance(result['X'], Decimal) else result['X'],
                    "y": float(result['Y']) if isinstance(result['Y'], Decimal) else result['Y'],
                    "z": float(result['Z']) if isinstance(result['Z'], Decimal) else result['Z'],
                    "w": float(result['W']) if isinstance(result['W'], Decimal) else result['W'],
                }
                return JSONResponse(content=response)  # 200 상태 코드로 자동 반환
            else:
                return JSONResponse(content={})  # 결과가 없을 경우 빈 응답 반환

    finally:
        conn.close()

@app.post("/start-guide")
async def start_guide(data: dict):  
    try:
        x = float(data.get("x"))
        y = float(data.get("y"))
        z = float(data.get("z"))
        w = float(data.get("w"))
        
        rclpy.init()
        status_queue = queue.Queue(maxsize=3)  
        subscriber_node = DetectionSubscriber(status_queue)
        navigation_node = NavigationNode(status_queue)
        
        goal_pose = navigation_node.create_goal_pose(x, y, z, w)
        navigation_node.goal_pose = goal_pose  
        executor = MultiThreadedExecutor()
        executor.add_node(subscriber_node)
        executor.add_node(navigation_node)
        try:
            executor.spin()
        except RuntimeError:
            pass
        finally:
            executor.shutdown()
            subscriber_node.destroy_node()
            navigation_node.destroy_node()
            rclpy.shutdown()

        return JSONResponse(content={"status": navigation_node.navigation_status})
    except (ValueError, TypeError) as e:
        raise HTTPException(status_code=400, detail="Invalid input data.")
    
@app.post("/end-guide")
async def end_guide():  
    try:
        rclpy.init()
        
        # ArUco 마커 제어 노드 초기화
        aruco_controller = ArUcoMarkerController()

        # 노드 실행
        executor = MultiThreadedExecutor()
        executor.add_node(aruco_controller)
        
        try:
            executor.spin()  # ArUco 제어 루프 실행
        except KeyboardInterrupt:
            aruco_controller.get_logger().info("ArUco 마커 제어 프로그램이 종료되었습니다.")
        finally:
            executor.shutdown()
            aruco_controller.destroy_node()
            rclpy.shutdown()

        return JSONResponse(content={"status": "Home station alignment completed"})
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to align with home station: {str(e)}")

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
