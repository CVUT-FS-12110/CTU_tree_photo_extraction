import joblib
import math
import numpy as np
from numpy import unique
from numpy import where
from sklearn.cluster import MeanShift, estimate_bandwidth
from matplotlib import pyplot as plt
import time
from geopy.distance import distance

#Měření času běhu programu
starttime = time.time()

res1 = 31
res2 = 112.7
#res2 = 546

gps_data = joblib.load('data_memory/gps_data_'+str(res1)+'_'+str(res2)+'.sav')
pose3d_data = joblib.load('data_memory/pose2d_data_'+str(res1)+'_'+str(res2)+'.sav')
lidar_data = joblib.load('data_memory/lidar_data_full_'+str(res1)+'_'+str(res2)+'.sav')

#lidar parametry a konstanty
dif_deg = 270 / 811
max_range = 1700
min_range = 400
# angle_range_low = 315
# angle_range_high = 540

angle_range_low= 45
angle_range_high= 240

sensor_placement = [250, 1200]
centlowlim = -150
centhighlim = 150
minclusterpoints = 10
maxclusterpoints = 120
ident_limit = 200

#fuzni parametry
weight_pose3d = 0.1
weight_gps = 0.9
def rotation_angle(point_1, point_2):
    #Výpocet vektoru
    vector_1 = (point_1[0], point_1[1])
    vector_2 = (point_2[0], point_2[1])
    # Skalarni soucin vektoru
    skal_souc = vector_1[0] * vector_2[0] + vector_1[1] * vector_2[1]
    # Velikost vektoru
    vector_1_lenght = math.sqrt(vector_1[0] ** 2 + vector_1[1] ** 2)
    vector_2_lenght = math.sqrt(vector_2[0] ** 2 + vector_2[1] ** 2)
    angle_rad = math.acos(skal_souc / (vector_1_lenght * vector_2_lenght))
    angle_deg = math.degrees(angle_rad)
    # Rozhodnuti o smeru rotace
    if vector_1[0] * vector_2[1] - vector_1[1] * vector_2[0] < 0:
        angle_deg = 360 - angle_deg
    return -angle_deg

def rotate(x, y, angle_deg): #Kladny smer po smeru hodinovych rucicek
    x_new = x*math.cos(math.radians(angle_deg)) - y*math.sin(math.radians(angle_deg))
    y_new = x*math.sin(math.radians(angle_deg)) + y*math.cos(math.radians(angle_deg))
    return [x_new, y_new]

fused_position=[0,0]
fused_trajectory=[]
#Pro vizualizaci
gps_position=[0,0]
pose3d_position=[0,0]
pose3dR_position=[0,0]
gps_trajectory=[]
pose3d_trajectory=[]
pose3dR_trajectory=[]

centerlist_lidar=[]

#Zacatecni a koncovy bod
start_gps = [gps_data[0][1][0], gps_data[0][1][1]]
start_pose3d = [pose3d_data[0][1][0], pose3d_data[0][1][1],pose3d_data[0][2]]
end_gps = [gps_data[-1][1][0], gps_data[-1][1][1]]
end_pose3d = [pose3d_data[-1][1][0], pose3d_data[-1][1][1],
              pose3d_data[-1][2]]
#Rozdil mezi zacatkem a koncem v jednotlivych osach
diff_end_gps = [distance([start_gps[1], start_gps[0]], [start_gps[1],
                gps_data[-1][1][0]]).m * 1000,
                distance([start_gps[1], start_gps[0]], [gps_data[-1][1][1],
                start_gps[0]]).m * 1000]
diff_end_pose3d = [pose3d_data[-1][1][0] - start_pose3d[0],
                   pose3d_data[-1][1][1] - start_pose3d[1],
                   pose3d_data[-1][2] - start_pose3d[2]]
#Vypocet uhlu rotace
ang_rot = rotation_angle([diff_end_gps[0], diff_end_gps[1]],
          [diff_end_pose3d[0], diff_end_pose3d[1]])

identifier = 0

#projizdim pres vsechny lidar snimky
for a in range(len(lidar_data)):
    print("Čas záznamu",lidar_data[a][0].total_seconds())
    merged_lidar_short_xy = []

    #posun vuci startu gps                 y             x               y             x
    diff_start_gps = [distance([start_gps[1], start_gps[0]], [start_gps[1], gps_data[a][1][0]]).m * 1000,
                      distance([start_gps[1], start_gps[0]], [gps_data[a][1][1], start_gps[0]]).m * 1000]
    #posun vuci startu pose3d
    diff_start_pose3d = [pose3d_data[a][1][0] - start_pose3d[0], pose3d_data[a][1][1]-start_pose3d[1],
                         pose3d_data[a][2] - start_pose3d[2]]

    #srovnani obou souradnicovych systemu
    if a > 1:
        #Rozdíly oproti předchozím hodnotám
        diff_prev_gps = [distance([gps_data[a-1][1][1], gps_data[a-1][1][0]],
                                        [gps_data[a-1][1][1], gps_data[a][1][0]]).m * 1000,
                         distance([gps_data[a-1][1][1], gps_data[a-1][1][0]],
                                        [gps_data[a][1][1], gps_data[a-1][1][0]]).m * 1000]
        diff_prev_pose3d = [pose3d_data[a][1][0] - pose3d_data[a-1][1][0],
                            pose3d_data[a][1][1] - pose3d_data[a-1][1][1],
                            pose3d_data[a][2] - pose3d_data[a-1][2]]
        diff_prev_pose3d_R = rotate(diff_prev_pose3d[0], diff_prev_pose3d[1], ang_rot)
        #Spojení odometrických a gps informací na základě váženého průměru
        fused_prev_move = [diff_prev_pose3d_R[0] * weight_pose3d + diff_prev_gps[0] * weight_gps,
                           diff_prev_pose3d_R[1] * weight_pose3d + diff_prev_gps[1] * weight_gps]

        fused_position = [fused_position[0] + fused_prev_move[0], fused_position[1] + fused_prev_move[1]]
        #Pripnuti finalni pozice do listu
        fused_trajectory.append(fused_position)

        #vizualizace fuse pozice
        gps_position = [gps_position[0] + diff_prev_gps[0], gps_position[1] + diff_prev_gps[1]]
        pose3d_position = [pose3d_position[0] + diff_prev_pose3d[0], pose3d_position[1] + diff_prev_pose3d[1]]
        pose3dR_position = [pose3dR_position[0] + diff_prev_pose3d_R[0], pose3dR_position[1] + diff_prev_pose3d_R[1]]
        gps_trajectory.append(gps_position)
        pose3d_trajectory.append(pose3d_position)
        pose3dR_trajectory.append(pose3dR_position)

    else:
        fused_position = [0, 0]
        gps_position = [0, 0]
        pose3d_position = [0, 0]
        pose3dR_position = [0, 0]

        fused_trajectory.append(fused_position)
        gps_trajectory.append(gps_position)
        pose3d_trajectory.append(pose3d_position)
        pose3dR_trajectory.append(pose3dR_position)

    A = lidar_data[a][1]
    # prevod polarnich dat na xy souradnice,omezene zvolenymi parametry - rozpeti uhlu a vzdalenosti
    #PRO LIDAR NATOČENÝ SMĚREM K ŘADĚ STROMŮ
    for l in range(angle_range_low, angle_range_high, 1):
        if min_range < A[l] < max_range:
            if (-45 <= -45 + dif_deg * l) & (-45 + dif_deg * l < 0):
                merged_lidar_short_xy.append([-A[l] * math.sin(math.radians(45 - dif_deg * l)),
                                              -A[l] * math.cos(math.radians(45 - dif_deg * l))])
            elif (0 < -45 + dif_deg * l) & (-45 + dif_deg * l < 90):
                merged_lidar_short_xy.append([A[l] * math.sin(math.radians(-45 + dif_deg * l)),
                                             -A[l] * math.cos(math.radians(-45 + dif_deg * l))])
            elif (90 < -45 + dif_deg * l) & (-45 + dif_deg * l < 180):
                merged_lidar_short_xy.append([A[l] * math.cos(math.radians(-135 + dif_deg * l)),
                                              A[l] * math.sin(math.radians(-135 + dif_deg * l))])
            elif (180 < -45 + dif_deg * l) & (-45 + dif_deg * l <= 225):
                merged_lidar_short_xy.append([-A[l] * math.sin(math.radians(-225 + dif_deg * l)),
                                              +A[l] * math.cos(math.radians(-225 + dif_deg * l))])
            else:
                print("Out of lidar FOV")
        else:
            continue
    """
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
    """
    # Mean-Shift clustering
    X = np.array(merged_lidar_short_xy)  # Definice datasetu
    bandwidth = estimate_bandwidth(X, quantile=0.25)
    model = MeanShift(bandwidth=bandwidth)
    yhat = model.fit_predict(X)
    clusters = unique(yhat)
    # Urceni souradnic stredu shluku
    cluster_centers = model.cluster_centers_
    # Nejvetsi velikost shluku
    cluster_size = max(unique(yhat, return_counts=True)[1])
    # Pokud je nejvetsi shluk moc velky, dalsi iterace
    if cluster_size > maxclusterpoints:
        continue

    # Iterace pres kazdy cluster
    for index, cluster in enumerate(clusters, start=0):
        row_ix_list = where(yhat == cluster)[0].tolist()  # List bodu jednoho shluku
        # Minimalni velikost shluku k analyze
        if len(row_ix_list) > minclusterpoints:
            # Omezeni se na oblast ve stredu snimku
            if (cluster_centers[index][1] > centlowlim) & (cluster_centers[index][1] < centhighlim):
                center = cluster_centers[index]
                # [lokal. centr. + umisteni senzoru    + pozice robota]
                real_center = [center[0] + sensor_placement[0] + fused_position[0],
                               center[1] + sensor_placement[1] + fused_position[1]]
                if len(centerlist_lidar) == 0:
                    # Prvni detekce - pouze pripneme
                    centerlist_lidar.append([a, identifier, [real_center[0], real_center[1]]])
                    # [ index a, identifikator, [centrum]]
                else:
                    # Predikce pozice posledniho ulozeneho centra na novem snimku
                    idd = centerlist_lidar[-1][1]
                    ref_dist = [fused_trajectory[a][0] - fused_trajectory[idd][0],
                                fused_trajectory[a][1] - fused_trajectory[idd][1]]
                    prev_center = centerlist_lidar[-1][2]
                    pred_center = [prev_center[0], prev_center[1]]
                    diff = math.dist(real_center, pred_center)
                    # Porovnani vzdalenosti mezi predikci a vysetrovanym centrem
                    if diff < ident_limit:
                        # Jedna se o posunute centrum, stejny identifikator
                        centerlist_lidar.append([a, identifier, [real_center[0], real_center[1]]])
                    else:
                        # Jedna se o nove centrum, novy identifikator
                        identifier = identifier + 1
                        centerlist_lidar.append([a, identifier, [real_center[0], real_center[1]]])
        else:
            continue
    else:
        continue
    # projizdim kazdy cluster

print(centerlist_lidar)
centersx = []
centersy = []
identifikator = 0
sumax = 0
sumay = 0
numsum = 0
total = []

#Tato cast probiha v dalsim skripu lidarmap_cleaner, tady jen pro vizualizaci
for v in range(len(centerlist_lidar)):
    centersx.append(centerlist_lidar[v][2][0])
    centersy.append(centerlist_lidar[v][2][1])
    if (centerlist_lidar[v][1] == identifikator):
        sumax = sumax+centerlist_lidar[v][2][0]
        sumay = sumay+centerlist_lidar[v][2][1]
        numsum = numsum+1
    else:
        identifikator = identifikator+1
        total.append([sumax/numsum, sumay/numsum])
        sumax = 0
        sumay = 0
        numsum = 0
        sumax = sumax + centerlist_lidar[v][2][0]
        sumay = sumay + centerlist_lidar[v][2][1]
        numsum = numsum + 1
print("total",total)

endtime = time.time()
print("ubehly cas", endtime - starttime)
#vizualizace
totalx,totaly = zip(*total)
plt.figure(figsize=(6, 6))
plt.title("Detekovana centra")
plt.scatter(centersx,centersy,s=5, c='m', marker='o', label='possible centers')
plt.scatter(totalx, totaly, s=10, c='k', marker='s', label='total')
plt.legend(loc='upper left', fontsize='large')
plt.grid(visible=True)
plt.xlabel("x [mm]")
plt.ylabel("y [mm]")
plt.show()


# vizualizace vysledne trajektorie
posex, posey = zip(*pose3d_trajectory)
poseRx, poseRy = zip(*pose3dR_trajectory)
gpsx, gpsy = zip(*gps_trajectory)
fusedx, fusedy = zip(*fused_trajectory)
plt.figure(figsize=(6, 6))
plt.title("Trajektorie")
plt.scatter(gpsx, gpsy, s=10, c='y', marker='o', label='GPS')
plt.scatter(posex, posey, s=10, c='r', marker='o', label='pose3d')
plt.scatter(poseRx, poseRy, s=10, c='b', marker='o', label='pose3d po rotaci')
plt.scatter(fusedx, fusedy, s=10, c='g', marker='o', label='výsledná fúze')
plt.grid(visible=True)
plt.xlabel("x [mm]")
plt.ylabel("y [mm]")
plt.legend(loc='upper left', fontsize='large')
plt.show()

#Meziukladani souboru detekovanych center a vysledne trajektorie
joblib.dump(centerlist_lidar,"data_memory/lidarmap_raw_"+str(res1)+'_'+str(res2)+".sav")
joblib.dump(fused_trajectory,"data_memory/fused_trajectory_2022_"+str(res1)+'_'+str(res2)+".sav")