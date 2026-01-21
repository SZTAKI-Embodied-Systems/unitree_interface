import motioncapture
import time as time_ext
import os
from pathlib import Path
import pickle

# mocap = motioncapture.MotionCaptureOptitrack("192.168.2.141")
mocap = motioncapture.connect(
    "optitrack",
    {'hostname': '192.168.2.141'}
)  
start_time = time_ext.time()   
log_opti = {"time": [],
                         "position": [],
                         "rotation": []}
counter = 0
while counter < 100:
    counter += 1
    mocap.waitForNextFrame()
    body = mocap.rigidBodies.get("go2")
    log_opti["time"].append(time_ext.time() - start_time)
    log_opti["position"].append(body.position)
    log_opti["rotation"].append([body.rotation.w, body.rotation.x, body.rotation.y, body.rotation.z])
    print(counter)
    print("Quat w:", float(body.rotation.w))
    # time_ext.sleep(0.0001)
path = Path("data/log_opti.json")
os.makedirs(path.parent, exist_ok=True)

with open("output/log_opti.pkl", "wb") as f:
    pickle.dump(log_opti, f)