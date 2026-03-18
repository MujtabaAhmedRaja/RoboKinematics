from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
import json
import os
import uvicorn
from kinematics import RobotArm

import models
from database import engine
from routers import auth, configs, trajectories

models.Base.metadata.create_all(bind=engine)

app = FastAPI()

app.include_router(auth.router, prefix="/api/auth", tags=["auth"])
app.include_router(configs.router, prefix="/api/configs", tags=["configs"])
app.include_router(trajectories.router, prefix="/api/trajectories", tags=["trajectories"])

frontend_dir = os.path.join(os.path.dirname(__file__), "..", "frontend")

# Ensure frontend directory exists
os.makedirs(frontend_dir, exist_ok=True)

app.mount("/static", StaticFiles(directory=frontend_dir), name="static")

@app.get("/")
async def read_index():
    index_path = os.path.join(frontend_dir, "index.html")
    if os.path.exists(index_path):
        return FileResponse(index_path)
    return {"error": "index.html not found in frontend directory"}

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    
    try:
        while True:
            data_str = await websocket.receive_text()
            data = json.loads(data_str)
            
            dof = data.get('dof', 3)
            dh_params = data.get('dh_params', [])
            mode = data.get('mode', 'fk')
            
            arm = RobotArm(dh_params)
            
            response = {"mode": mode}
            
            if mode == 'fk':
                angles = data.get('joint_angles', [0]*dof)
                frames = arm.forward_kinematics(angles)
                response['frames'] = [f.tolist() for f in frames]
                response['angles'] = angles
                
            elif mode == 'ik':
                target = data.get('target', {'x':0, 'y':0, 'z':0})
                target_pos = [target['x'], target['y'], target['z']]
                current_angles = data.get('joint_angles', [0]*dof)
                locked_joints = data.get('locked_joints', [False]*dof)
                
                # We use the robust numerical ik to avoid issues with arbitrary DH params
                new_angles = arm.inverse_kinematics_numerical(target_pos, current_angles, locked_joints)
                frames = arm.forward_kinematics(new_angles)
                
                response['angles'] = new_angles
                response['frames'] = [f.tolist() for f in frames]

            elif mode == 'workspace':
                pts = arm.generate_workspace(3000)
                await websocket.send_text(json.dumps({"workspace_points": pts}))
                continue
            
            await websocket.send_text(json.dumps(response))
            
    except WebSocketDisconnect:
        pass

if __name__ == "__main__":
    uvicorn.run("main:app", host="127.0.0.1", port=8000, reload=True)
