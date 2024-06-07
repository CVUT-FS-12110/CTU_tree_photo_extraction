import joblib
import numpy as np
import math
from matplotlib import pyplot as plt
import config

res1 = config.res1
res2 = config.res2


all_trees = joblib.load("data_memory/cameramap_raw_"+str(res1)+'_'+str(res2)+'.sav')
centerlist_arecont_noid = joblib.load('data_memory/cameramap_raw_noid_'+str(res1)+'_'+str(res2)+'.sav')

allx = []
ally = []
ids = []

#Nalezeni nejblizsiho bodu v listu
def find_nearest_point(target, list):
    nearest_point = list[0]
    min_dist = math.dist(target, nearest_point)
    for point in list[1:]:
        dist = math.dist(target, point)
        if dist < min_dist:
            min_dist = dist
            nearest_point = point
    return nearest_point, min_dist

for oo in range(len(all_trees)):
    #jen pro vizualizaci
    allx.append(all_trees[oo][2][0])
    ally.append(all_trees[oo][2][1])
    ids.append(all_trees[oo][1])

    confbox = all_trees[oo][0]
    boxid = all_trees[oo][1]
    treex = all_trees[oo][2][0]
    treey = all_trees[oo][2][1]

noidx = []
noidy = []
for uu in range(len(centerlist_arecont_noid)):
    noidx.append(centerlist_arecont_noid[uu][0])
    noidy.append(centerlist_arecont_noid[uu][1])


maxid = int(max(np.array(ids, dtype=np.float64)))
print(maxid)

final_list = []

sumx = 0
sumy = 0
numsum = 0
confsum = 0
timersum = 0

# kvuli limitovanemu oknu mam vzdy jen 1 strom v trackingu, ale tento pristup pocita i s pripadem, kde jsou ids prohazena
for cc in range(maxid+1):
    for ll in range(len(all_trees)):
        confbox = all_trees[ll][0]
        boxid = all_trees[ll][1]
        treex = all_trees[ll][2][0]
        treey = all_trees[ll][2][1]
        timerc = all_trees[ll][3]
        if boxid == cc:
            sumx = sumx + treex*confbox
            sumy = sumy + treey*confbox
            numsum = numsum + 1
            confsum = confsum + confbox
            timersum = timersum + timerc*confbox
        else:
            continue
    if numsum == 0:
        pass
    else:
        final_list.append([timersum/confsum, [sumx/confsum, sumy/confsum]])
    sumx = 0
    sumy = 0
    numsum = 0
    confsum = 0
    timersum = 0

#Vizualizace
finalx = []
finaly = []
for ii in range(len(final_list)):
    finalx.append(final_list[ii][1][0])
    finaly.append(final_list[ii][1][1])

plt.figure(figsize=(6, 6))
plt.title("Mapa sadu z kamerového záznamu")
plt.xlabel("x [mm]")
plt.ylabel("y [mm]")
plt.scatter(allx, ally, s=7, c='y', marker='o', label='Detekce s ID')
plt.scatter(noidx, noidy, s=7, c='r', marker='o', label='Detekce bez ID')
plt.scatter(finalx, finaly, s=10, c='k', marker='x', label='Výsledná mapa')
plt.legend(loc='upper left')
plt.show()

joblib.dump(final_list, "data_memory/cameramap_clean_"+str(res1)+"_"+str(res2)+".sav")

