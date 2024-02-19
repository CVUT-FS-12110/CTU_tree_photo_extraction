import joblib
import numpy as np
import math
from matplotlib import pyplot as plt

res1 = 31
res2 = 112.7
#res2 = 546


amap_poles = joblib.load("data_memory/amap_poles_"+str(res1)+'_'+str(res2)+'.sav')
amap_trees = joblib.load("data_memory/amap_trees_"+str(res1)+'_'+str(res2)+'.sav')
center_lidar = joblib.load("data_memory/lidarmap_clean_"+str(res1)+"_"+str(res2)+".sav")
center_arecont = joblib.load("data_memory/cameramap_clean_"+str(res1)+"_"+str(res2)+".sav")
fused_trajectory = joblib.load('data_memory/fused_trajectory_'+str(res1)+'_'+str(res2)+'.sav')

amap = amap_trees
lidar_weight = 0.3
camera_weight = 0.3
amap_weight = 0.4

merge_limit = 200
missing_trees = 0

lidarx = []
lidary = []
lidar_time = []
for ff in range(len(center_lidar)):
    lidarx.append(center_lidar[ff][1][0])
    lidary.append(center_lidar[ff][1][1])
    lidar_time.append(center_lidar[ff][0])

arecontx = []
areconty = []
arecont_time = []
for hh in range(len(center_arecont)):
    arecontx.append(center_arecont[hh][1][0])
    areconty.append(center_arecont[hh][1][1])
    arecont_time.append(center_arecont[hh][0])

print("lidar_time", lidar_time)
print("arecont_time", arecont_time)


polesx, polesy = zip(*amap_poles)
treesx, treesy = zip(*amap_trees)
trajectx, trajecty = zip(*fused_trajectory)


print((len(amap_poles)+len(amap_trees[0])))
print(len(center_lidar))
print(len(center_arecont))

plt.figure(figsize=(6, 6))
plt.title("Vstupní data")
plt.scatter(treesx, treesy, s=10, c='b', marker='o', label='amap')
plt.scatter(polesx, polesy, s=10, c='m', marker='o', label='sloupy')
plt.scatter(lidarx, lidary, s=10, c='k', marker='x', label='lidar_map')
plt.scatter(arecontx, areconty, s=10, c='g', marker='x', label='camera_map')
plt.scatter(trajectx, trajecty, s=10, c='y', marker='o', label='trajektorie robota')
plt.legend(loc='upper left', fontsize='large')
plt.xlabel("x [mm]")
plt.ylabel("y [mm]")
plt.show()

#hledam nejblizsi
closest_list_lidar = []
closest_list_lidar_t = []
closest_list_arecont = []
closest_list_arecont_t =[]

nonidentified_lidar = []
nonidentified_lidar_t = []
nonidentified_camera = []
nonidentified_camera_t = []
print(lidarx)
print(amap)
print(lidary)

for l in range(len(amap)):
    # Lidarova cast
    closest_l = 1000000
    closest_l_ind = 1000000
    # Ukladani nejkratsi vzdalenosti ke kontrolovanemu bodu
    for i in range(len(center_lidar)):
        dd = math.dist(amap[l], [lidarx[i], lidary[i]])
        if dd < closest_l:
            closest_l = dd
            closest_l_ind = i

    if closest_l > merge_limit:
        # Pokud je nejkratsi vzdalenost vetsi nez limit
        print("Neidentifikovano")
        # Pripnuti np.nan do listu souradnic a casu
        closest_list_lidar.append([np.nan, np.nan])
        closest_list_lidar_t.append([np.nan, np.nan])
        nonidentified_lidar.append([center_lidar[closest_l_ind][1][0],
                                    center_lidar[closest_l_ind][1][1]])
        nonidentified_lidar_t.append(center_lidar[closest_l_ind][0])

    else:
        # Pripnuti nejblizsiho centra a casu do listu
        closest_list_lidar.append([center_lidar[closest_l_ind][1][0],
                                   center_lidar[closest_l_ind][1][1]])
        closest_list_lidar_t.append(center_lidar[closest_l_ind][0])

    # kamera cast
    closest_a = 100000
    closest_a_ind = 100000
    # Ukladani nejkratsi vzdalenosti ke kontrolovanemu bodu
    for k in range(len(center_arecont)):
        ss = math.dist(amap[l], [arecontx[k], areconty[k]])
        if ss < closest_a:
            closest_a = ss
            closest_a_ind = k
    if closest_a > merge_limit:
        # Pokud je nejkratsi vzdalenost vetsi nez limit
        print("Neidentifikovano")
        # Pripnuti np.nan do listu souradnic a casu
        closest_list_arecont.append([np.nan, np.nan])
        closest_list_arecont_t.append([np.nan, np.nan])
        nonidentified_camera.append([center_arecont[closest_a_ind][1][0],center_arecont[closest_a_ind][1][1]])
        nonidentified_camera_t.append(center_arecont[closest_a_ind][0])
    else:
        closest_list_arecont.append([center_arecont[closest_a_ind][1][0],center_arecont[closest_a_ind][1][1]])
        closest_list_arecont_t.append(center_arecont[closest_a_ind][0])
print(amap)
print(closest_list_lidar)
print(closest_list_arecont)

print(len(amap))
print(len(closest_list_lidar))
print(len(closest_list_arecont))

fused_map = []
warning = 0
for kk in range(len(amap)):
    cam_value = closest_list_arecont[kk]
    lid_value = closest_list_lidar[kk]
    amap_value = amap[kk]

    cam_time = closest_list_arecont_t[kk]
    lid_time = closest_list_lidar_t[kk]

    if not np.isnan(cam_value[0]) and not np.isnan(lid_value[0]) and not np.isnan(amap_value[0]):
        print("Identifikace všemi třemi způsoby")
        #Vážený průměr odpovídajících si bodů
        cent = [amap_value[0] * amap_weight + lid_value[0] * lidar_weight + cam_value[0] * camera_weight,
                amap_value[1] * amap_weight + lid_value[1] * lidar_weight + cam_value[1] * camera_weight]
        #Vážený průměr času
        timer = (cam_time * camera_weight + lid_time * lidar_weight) / (camera_weight + lidar_weight)
        #Připnutí času a pozice finálního stromu
        fused_map.append([timer, cent])

    elif np.isnan(cam_value[0]) and not np.isnan(lid_value[0]) and not np.isnan(amap_value[0]):
        print("Identifikovan strom, pouze lidarem")
        #Vážený průměr odpovídajících si bodů
        cent = [(amap_value[0] * amap_weight + lid_value[0] * lidar_weight)/(amap_weight+lidar_weight),
                (amap_value[1] * amap_weight + lid_value[1] * lidar_weight)/(amap_weight+lidar_weight)]
        timer = lid_time
        #Připnutí času a pozice finálního stromu
        fused_map.append([timer, cent])

    elif not np.isnan(cam_value[0]) and np.isnan(lid_value[0]) and not np.isnan(amap_value[0]):
        print("Strom identifikovany kamerou, ale neurceny lidarem")
        #Vážený průměr odpovídajících si bodů
        cent = [(amap_value[0] * amap_weight + cam_value[0] * camera_weight)/(amap_weight+camera_weight),
                (amap_value[1] * amap_weight + cam_value[1] * camera_weight)/(amap_weight+camera_weight)]
        timer = cam_time
        #Připnutí času a pozice finálního stromu
        fused_map.append([timer, cent])


    elif np.isnan(cam_value[0]) and np.isnan(lid_value[0]) and not np.isnan(amap_value[0]):
        print("Strom určený pouze apriorní mapou")
        missing_trees += 1
        print("2/3  failed")

    else:
        print("2/3  failed")

noncx, noncy = zip(*nonidentified_camera)
nonlx, nonly = zip(*nonidentified_lidar)



fusedx = []
fusedy = []
fusedtime=[]
for nn in range(len(fused_map)):
    fusedx.append(fused_map[nn][1][0])
    fusedy.append(fused_map[nn][1][1])
    fusedtime.append(fused_map[nn][0])

plt.figure(figsize=(6, 6))
plt.title("Finální mapa")
plt.scatter(trajectx, trajecty, s=10, c='y', marker='s', label='trajektorie robota')
plt.scatter(fusedx, fusedy, s=20, c='r', marker='s', label='výsledná mapa stromů')
plt.scatter(polesx, polesy, s=20, c='k', marker='x', label='sloupy')
plt.legend(loc='upper left', fontsize='large')
plt.xlabel("x [mm]")
plt.ylabel("y [mm]")

plt.show()

#Kontrola nepřiřazených bodů mapy
def find_nearest_point(target, list):
    nearest_point = list[0]
    min_dist = math.dist(target, nearest_point)
    for point in list[1:]:
        dist = math.dist(target, point)
        if dist < min_dist:
            min_dist = dist
            nearest_point = point
    return nearest_point, min_dist

plt.figure(figsize=(6,6))
plt.title("Nepřiřazená měření")
plt.scatter(noncx, noncy, s=10, c='y', marker='s', label='no id kamera')
plt.scatter(nonlx, nonly, s=20, c='r', marker='s', label='no id lidar')

#Pokud jsou nějaké dva body z kamerové mapy a lidarové mapy blízko sebe - varování, možné špatné přiřazení stromu
if len(nonidentified_camera) > len(nonidentified_lidar):
    rang = len(nonidentified_camera)
    for hi in range(rang):
        nearest_point, close = find_nearest_point(nonidentified_camera[hi],nonidentified_lidar)
        if close < merge_limit:
            print("varování v čase", nonidentified_camera_t[hi])
else:
    rang = len(nonidentified_lidar)
    for hi in range(rang):
        nearest_point, close = find_nearest_point(nonidentified_lidar[hi],nonidentified_camera)
        if close < merge_limit:
            print("varování v čase", nonidentified_lidar_t[hi])
print(missing_trees)
plt.show()
#joblib.dump(fused_map,"data_memory/fused_map_"+str(res1)+"_"+str(res2)+".sav")
#joblib.dump(missing_trees, "data_memory/missing_trees_"+str(res1)+"_"+str(res2)+".sav")

