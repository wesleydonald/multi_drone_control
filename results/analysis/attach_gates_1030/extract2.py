import os, sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
OUT=os.path.dirname(os.path.abspath(__file__))
bag, tag = sys.argv[1], sys.argv[2]
db=[os.path.join(bag,f) for f in os.listdir(bag) if f.endswith('.db3')][0]
cur=sqlite3.connect(db).cursor()
tops={n:(i,t) for i,n,t in cur.execute('select id,name,type from topics')}
data={}
for name in ['/payload/motion_capture_state']+[f'/drone_{i}/magnet_tip_state' for i in range(4)]+[f'/drone_{i}/motion_capture_state' for i in range(4)]:
    if name not in tops: continue
    i,ty=tops[name]; cls=get_message(ty); rows=[]
    for ts,d in cur.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,)):
        m=deserialize_message(d,cls); p,q=m.pose.position,m.pose.orientation
        rows.append((ts*1e-9,p.x,p.y,p.z,q.x,q.y,q.z,q.w))
    data[name.strip('/').replace('/','_')]=np.array(rows)
np.savez(os.path.join(OUT,f'{tag}_sq.npz'),**data); print(tag,{k:len(v) for k,v in data.items()})
