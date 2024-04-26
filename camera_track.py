import cv2
from ultralytics import YOLO
import joblib
import torch
import json
import numpy as np
import math
from numpy import unique
from sklearn.cluster import MeanShift, estimate_bandwidth
from statistics import mean
from matplotlib import pyplot as plt
import time

res1 = 31
res2 = 112.7
#res2 = 546
starttime = time.time()

gps_data = joblib.load('data_memory/gps_data_'+str(res1)+'_'+str(res2)+'.sav')
pose3d_data = joblib.load('data_memory/pose2d_data_'+str(res1)+'_'+str(res2)+'.sav')
lidar_data = joblib.load('data_memory/lidar_data_full_'+str(res1)+'_'+str(res2)+'.sav')

rscolor_data = joblib.load('data_memory/rscolor_data_full_'+str(res1)+'_'+str(res2)+'.sav')
arecontcam_data = joblib.load('data_memory/arecontcam_data_full_'+str(res1)+'_'+str(res2)+'.sav')
fused_trajectory = joblib.load('data_memory/fused_trajectory_2022_'+str(res1)+'_'+str(res2)+'.sav')



#  if torch.cuda is available, set cuda device
if torch.cuda.is_available():
    torch.cuda.set_device(0)

dif_deg = 270 / 811
max_range = 1700
min_range = 700
angle_range_low = 0
angle_range_high = 135

distortion_limit = 200
centlowlim = -150
centhighlim = 150

arecont_res = [1024, 768]
sensor_placement = [250, 1200]
ident_limit = 200

def find_nearest(array, value):
    #vraci index nejblizsi vyssi hodnoty
    idx = np.searchsorted(array, value, side="left")
    if idx > 0 and (idx == len(array) or (value - array[idx-1]) < (value - array[idx])):
        #print(idx - 1)
        return idx-1
    else:
        #print(idx)
        return idx


def generate_y(x, picture_distance):
    y = -(x / arecont_res[1]) * picture_distance + picture_distance / 2
    return y


#Část interpolace a dopočítání pozice z fused_trajectory pro časová razítka kamerových měření
fused_trajectory_t = []
arecontcam_t = []
rscolor_t = []
position_arecontcam = []
position_rscolor = []
for b in range(len(lidar_data)):
    fused_trajectory_t.append(lidar_data[b][0].total_seconds())
for c in range(len(arecontcam_data)):
    arecontcam_t.append(arecontcam_data[c][0].total_seconds())
for d in range(len(rscolor_data)):
    rscolor_t.append(rscolor_data[d][0].total_seconds())

for m in range(len(arecontcam_t)):
    interpol_pose_arecont = []
    interpol_pose_arecont.clear()
    fused_trajectory_ind = find_nearest(array=fused_trajectory_t, value=arecontcam_t[m])

    for p in range(len(fused_trajectory[0])):
        interpol_pose_arecont_1 = fused_trajectory[fused_trajectory_ind-1][p] + (arecontcam_t[m]-fused_trajectory_t[fused_trajectory_ind-1])*((fused_trajectory[fused_trajectory_ind][p]-fused_trajectory[fused_trajectory_ind-1][p])/(fused_trajectory_t[fused_trajectory_ind]-fused_trajectory_t[fused_trajectory_ind-1]))
        interpol_pose_arecont.append(interpol_pose_arecont_1)
    position_arecontcam.append(interpol_pose_arecont)

for n in range(len(rscolor_t)):
    interpol_pose_route = []
    interpol_pose_route.clear()
    route_trajectory_ind = find_nearest(array=fused_trajectory_t, value=rscolor_t[n])

    for o in range(len(fused_trajectory[0])):
        interpol_pose_route_1 = fused_trajectory[route_trajectory_ind-1][o]+(rscolor_t[n]-fused_trajectory_t[route_trajectory_ind-1])*((fused_trajectory[route_trajectory_ind][o]-fused_trajectory[route_trajectory_ind-1][o])/(fused_trajectory_t[route_trajectory_ind]-fused_trajectory_t[route_trajectory_ind-1]))
        interpol_pose_route.append(interpol_pose_route_1)
    position_rscolor.append(interpol_pose_route)

S = 4.8 #sirka senzoru
f = 1.2 #ohniskova vzdalenost
fov = math.atan(S/2*f)

all_trees = []
noid_trees = []
customid = 999

"""
#cast korekce soudkovitosti - u techto dat urizne cast kmene, takze nevhodne pro detekci
with open("./arecont_czu.json","r") as file:
    arc_conf=json.load(file)
with open("./routecam_cvut.json", "r") as file2:
    rout_conf=json.load(file2)
arc_mtx=np.array(arc_conf["camera_matrix"])
arc_dist=np.array(arc_conf["dist_coeff"])

#arc_mtx=np.array(rout_conf["camera_matrix"])
#arc_dist=np.array(rout_conf["dist_coeff"])
"""

model = YOLO("data_memory/yolov8.pt")

for a in range(len(arecontcam_data)):
    frame = arecontcam_data[a][1] #Nacteny snimek
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE) #Rotace snimku

    lid_ind = find_nearest(fused_trajectory_t, arecontcam_data[a][0].total_seconds())
    A = lidar_data[lid_ind][1]
    merged_lidar_short_xy = []
    # for l in range(angle_range_low, angle_range_high, 1):
    #     if min_range < A[l] < max_range:
    #         if (-45 <= -45 + dif_deg * l) & (-45 + dif_deg * l < 0):
    #             merged_lidar_short_xy.append([-A[l] * math.sin(math.radians(45 - dif_deg * l)),
    #                                           -A[l] * math.cos(math.radians(45 - dif_deg * l))])
    #         elif (0 < -45 + dif_deg * l) & (-45 + dif_deg * l < 90):
    #             merged_lidar_short_xy.append([A[l] * math.sin(math.radians(-45 + dif_deg * l)),
    #                                           -A[l] * math.cos(math.radians(-45 + dif_deg * l))])
    #         elif (90 < -45 + dif_deg * l) & (-45 + dif_deg * l < 180):
    #             merged_lidar_short_xy.append([A[l] * math.cos(math.radians(-135 + dif_deg * l)),
    #                                           A[l] * math.sin(math.radians(-135 + dif_deg * l))])
    #         elif (180 < -45 + dif_deg * l) & (-45 + dif_deg * l <= 225):
    #             merged_lidar_short_xy.append([-A[l] * math.sin(math.radians(-225 + dif_deg * l)),
    #                                           +A[l] * math.cos(math.radians(-225 + dif_deg * l))])
    #         else:
    #             print("Out of lidar FOV")
    #     else:
    #         continue


        #PRO LIDAR NATOČENÝ VE SMĚRU JÍZDY ROBOTA
    for l in range(angle_range_low, angle_range_high, 1):
        if (min_range < A[l] < max_range):
            if (-45 <= -45 + dif_deg * l) & (-45 + dif_deg * l < 0):
                merged_lidar_short_xy.append([+A[l] * math.cos(math.radians(45 - dif_deg * l)),
                                              -A[l] * math.sin(math.radians(45 - dif_deg * l))])

            elif (0 < -45 + dif_deg * l) & (-45 + dif_deg * l < 90):
                merged_lidar_short_xy.append([+A[l] * math.cos(math.radians(-45 + dif_deg * l)),
                                              +A[l] * math.sin(math.radians(-45 + dif_deg * l))])

            elif (90 < -45 + dif_deg * l) & (-45 + dif_deg * l < 180):
                merged_lidar_short_xy.append([-A[l] * math.sin(math.radians(-135 + dif_deg * l)),
                                              +A[l] * math.cos(math.radians(-135 + dif_deg * l))])

            elif (180 < -45 + dif_deg * l) & (-45 + dif_deg * l <= 225):
                merged_lidar_short_xy.append([-A[l] * math.cos(math.radians(-225 + dif_deg * l)),
                                              -A[l] * math.sin(math.radians(-225 + dif_deg * l))])

            else:
                print("Out of lidar FOV")
        else:
            continue

    X = np.array(merged_lidar_short_xy)  # Definice datasetu
    bandwidth = estimate_bandwidth(X, quantile=0.25)
    model2 = MeanShift(bandwidth=bandwidth)
    yhat = model2.fit_predict(X)
    clusters = unique(yhat)
    # Urceni souradnic stredu shluku
    cluster_centers = model2.cluster_centers_
    # Nejvetsi velikost shluku
    xlid, ylid = zip(*cluster_centers)

    p = mean(xlid)
    picture_distance = 2 * p * math.tan(fov / 2)

    #Vysledek detekce a sledovani
    results = model.track(frame, persist=True, conf=0.25, iou=0.5)
    #Vizualizace vysledku
    annotated_frame = results[0].plot()
    #Pixelove souradnice detekovaneho boxu
    boxes = results[0].boxes.xyxy
    for m in range(len(boxes)):
        #X souradnice stredu boxu
        xbox = float(boxes[m][0]+boxes[m][2])/2
        #Parametr duvery v detekci
        confbox = results[0].boxes.conf[m].item()
        #Identifikator stromu
        boxid = results[0].boxes.id
        #Omezeni se na stred stnimku
        if xbox > distortion_limit and xbox < arecont_res[1]-distortion_limit:
            if boxid != None:
                boxid = int(results[0].boxes.id[m].item())
                #Souradnice stromu
                tree_coord = [p + sensor_placement[0] + position_arecontcam[a][0],
                              generate_y(xbox, picture_distance) + sensor_placement[1]
                              + position_arecontcam[a][1]]
                cur = [confbox, boxid, [tree_coord[0], tree_coord[1]],
                       arecontcam_data[a][0].total_seconds(), a]
                all_trees.append(cur)
            else:
                #Situace, kdy strom nema prideleny identifikator
                tree_coord = [p + sensor_placement[0] + position_arecontcam[a][0],
                              generate_y(xbox, picture_distance) + sensor_placement[1] + position_arecontcam[a][1]]
                noid_trees.append([tree_coord[0], tree_coord[1]])
                
                if len(all_trees) == 0:
                    id = 0
                    prev_a = a
                    prev_center = [0, 0]
                    pred_center = [0, 0]

                else:
                    id = all_trees[-1][1]
                    prev_a = all_trees[-1][4]
                    prev_center = all_trees[-1][2]
                    pred_center = [prev_center[0], prev_center[1]]


                
                ref_dist = [position_arecontcam[a][0] - position_arecontcam[prev_a][0],
                            position_arecontcam[a][1] - position_arecontcam[prev_a][1]]
                
                #Vzdálenost predikované pozice centra a detekované pozice centra
                diff = math.dist(tree_coord, pred_center)

                if diff > ident_limit:
                    #pokud je vzdálenost větší - nový strom, nový identifikátor
                    customid = customid + 1
                    cur = [confbox, customid, [tree_coord[0], tree_coord[1]], arecontcam_data[a][0].total_seconds(), a]
                    all_trees.append(cur)
                else:
                    #existující strom, stejný identifikátor
                    #určení, zdali se jedná o centrum určené yolov8 nebo pro správný identifikátor
                    if id > 1000:
                        cur = [confbox, customid, [tree_coord[0], tree_coord[1]], arecontcam_data[a][0].total_seconds(),
                               a]
                        all_trees.append(cur)
                    else:
                        cur = [confbox, id, [tree_coord[0], tree_coord[1]], arecontcam_data[a][0].total_seconds(), a]
                        all_trees.append(cur)
    else:
            print("Mimo rozsah ohraniceni")
    """
    #cast korekce soudkovitosti
    h,w=frame.shape[:2]
    newcameratx, roi = cv2.getOptimalNewCameraMatrix(arc_mtx,arc_dist,(w,h), 1, (w,h))
    frame=cv2.undistort(frame,arc_mtx,arc_dist,None, newcameratx)
    x,y,w,h = roi
    frame=frame[y:y+h,x:x+w]
    #frame=frame[675:1024,0:768]
    """

    #Vizualizace detekce
    cv2.imshow("Camera_track", annotated_frame)
    cv2.waitKey(1)

#Výsledná vizualizace
allx = []
ally = []
ids = []
for oo in range(len(all_trees)):
    #jen pro vizualizaci
    allx.append(all_trees[oo][2][0])
    ally.append(all_trees[oo][2][1])
    ids.append(all_trees[oo][1])

    confbox = all_trees[oo][0]
    boxid = all_trees[oo][1]
    treex = all_trees[oo][2][0]
    treey = all_trees[oo][2][1]

endtime = time.time()
print("ubehly cas", endtime - starttime)

#Vizualizace
noidx, noidy = zip(*noid_trees)
plt.figure(figsize=(6, 6))
plt.title("Detekované kmeny")
plt.xlabel("x [mm]")
plt.ylabel("y [mm]")
plt.scatter(allx, ally, s=10, c='y', marker='o', label='yolo identifikace')
plt.scatter(noidx, noidy, s=10, c='r',marker='o', label='lidar identifikace')
plt.legend(loc='upper left')
plt.show()

#ukladani center co nemaji id z yolov8 sledovani pro vizualizaci
joblib.dump(all_trees,"data_memory/cameramap_raw_"+str(res1)+'_'+str(res2)+'.sav')
joblib.dump(noid_trees,"data_memory/cameramap_raw_noid_"+str(res1)+"_"+str(res2)+".sav")

