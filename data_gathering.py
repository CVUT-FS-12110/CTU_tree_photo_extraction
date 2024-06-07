import numpy as np
import cv2
from osgar.lib.serialize import deserialize
from osgar.logger import LogReader, lookup_stream_id
import pandas as pd
import math
import joblib
import config


#Načtení cesty k logu CONFIG
#log_file = 'data/ipc-dynamic-meas-240419_210554.log'
log_file = config.log_file

print(log_file)

#Definice názvů jednotlivých kanálů pro čtení.
#POZOR - data z roku 2022 a 2023 mají jiné názvy, zkontrolovat pomocí python3 -m osgar.logger logfile.log
sname_gps_position = "from_spider.position"
sname_realsense_color = "route_cam.image" #místo realsens to je routka
# sname_realsense_color = "realsense.color"
# sname_realsense_depth = "realsense.depth"
sname_lidar_scan = "from_spider.lidar_scan"
sname_from_spider_pose3d = "from_spider.pose3d"
sname_from_spider_pose2d = "from_spider.pose2d"
sname_arecontcam = "arecont.image"
# sname_arecontcam = "route_cam.image"

ostream_gps_position = lookup_stream_id(log_file, sname_gps_position)
ostream_realsense_color = lookup_stream_id(log_file, sname_realsense_color)
# ostream_realsense_depth = lookup_stream_id(log_file, sname_realsense_depth)
ostream_lidar_scan = lookup_stream_id(log_file, sname_lidar_scan)
ostream_from_spider_pose3d = lookup_stream_id(log_file, sname_from_spider_pose3d)
ostream_from_spider_pose2d = lookup_stream_id(log_file, sname_from_spider_pose2d)
ostream_arecontcam = lookup_stream_id(log_file,sname_arecontcam)

gps_data_full = []
pose2d_data_full = []
lidar_data_full = []
pose3d_data_full = []
rscolor_data_full = []
arecontcam_data_full = []
rsdepth_data_full = []

# Data o rotaci jsou uložena ve formě quaternionu
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians

#Zvolené časové rozmezi z logu ... CONFIG
res1 = config.res1
res2 = config.res2

lowertime = pd.Timedelta(seconds=res1)
uppertime = pd.Timedelta(seconds=res2)

#Argument, které kanály se mají číst
arg=[ostream_from_spider_pose2d,
     ostream_gps_position,
     ostream_lidar_scan,
     ostream_arecontcam,
     ostream_realsense_color
     ]

#Čtení jednotlivých dat na základě stream_id:
with LogReader(log_file, only_stream_id=arg) as log:
    for timestamp, stream_id, data in log:
        #Časové omezení pro čtení pouze určitého časového intervalu z logu
        if pd.Timedelta(timestamp) < lowertime:
            continue
        elif pd.Timedelta(timestamp) > uppertime:
            break

        if stream_id == ostream_realsense_color:
            buf_color = deserialize(data)
            color_im = cv2.imdecode(np.frombuffer(buf_color, dtype=np.uint8), 1)
            rscolor_data_full.append([timestamp, color_im])

        # if stream_id == ostream_realsense_depth:
        #     buf_depth = deserialize(data)
        #     color_im = cv2.imdecode(np.frombuffer(buf_depth, dtype=np.uint8), 1)

        if stream_id == ostream_gps_position:
            gps_position_ms = deserialize(data)
            #[east,north]
            gps_position = [gps_position_ms[0]*0.0000002777778, gps_position_ms[1]*0.0000002777778]
            gps_data_full.append([timestamp, gps_position])

        if stream_id == ostream_from_spider_pose2d:
            buf_pose2d = deserialize(data)
            #[x,y,yaw]
            from_spider_pose2d = (buf_pose2d[0], buf_pose2d[1], buf_pose2d[2]/100)
            pose2d_data_full.append([timestamp, from_spider_pose2d])

        if stream_id == ostream_from_spider_pose3d:
            buf_pose3d = deserialize(data)
            a, b, c = euler_from_quaternion(buf_pose3d[1][0], buf_pose3d[1][1],
                                            buf_pose3d[1][2], buf_pose3d[1][3])
            #[x,y,z,yaw]
            from_spider_pose3d = (buf_pose3d[0][0]*1000,buf_pose3d[0][1]*1000,buf_pose3d[0][2]*1000,c*180/math.pi)
            pose3d_data_full.append(([timestamp, from_spider_pose3d]))

        if stream_id == ostream_lidar_scan:
            buf_lidar_scan = deserialize(data)
            lidar_data_full.append([timestamp, buf_lidar_scan])

        if stream_id == ostream_arecontcam:
            buf_arecontcam = deserialize(data)
            arecontcam_im = cv2.imdecode(np.frombuffer(buf_arecontcam, dtype=np.uint8), 1)
            arecontcam_data_full.append([timestamp, arecontcam_im])
            print(timestamp)


#print("tady jsou delky dat")
#print("gps data", len(gps_data_full))
#print("pose2d data", len(pose2d_data_full))
#print("pose3d data", len(pose3d_data_full))
#print("lidar data", len(lidar_data_full))
#print("rscolor data", len(rscolor_data_full))
#print("arecontcam data", len(arecontcam_data_full))

print(res1, res2)

#Meziukládání do složky v projektu pro další využití. (soubor, path)
joblib.dump(gps_data_full,'data_memory/gps_data_full_'+str(res1)+'_'+str(res2)+'.sav')
joblib.dump(pose2d_data_full,'data_memory/pose2d_data_full_'+str(res1)+'_'+str(res2)+'.sav')
joblib.dump(pose3d_data_full,'data_memory/pose3d_data_full_'+str(res1)+'_'+str(res2)+'.sav')
joblib.dump(lidar_data_full,'data_memory/lidar_data_full_'+str(res1)+'_'+str(res2)+'.sav')
joblib.dump(rscolor_data_full,'data_memory/rscolor_data_full_'+str(res1)+'_'+str(res2)+'.sav')
joblib.dump(rsdepth_data_full,'data_memory/rsdepth_data_full_'+str(res1)+'_'+str(res2)+'.sav')
joblib.dump(arecontcam_data_full,'data_memory/arecontcam_data_full_'+str(res1)+'_'+str(res2)+'.sav')
