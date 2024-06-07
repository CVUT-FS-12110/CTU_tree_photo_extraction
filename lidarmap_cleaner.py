import joblib
from matplotlib import pyplot as plt
import config

res1 = config.res1
res2 = config.res2


centerlist_lidar = joblib.load("data_memory/lidarmap_raw_"+str(res1)+'_'+str(res2)+'.sav')
lidar_data = joblib.load('data_memory/lidar_data_full_'+str(res1)+'_'+str(res2)+'.sav')

lidart, lidard=zip(*lidar_data)

indexes = []
wrong_detections = []
right_detections = []
correct_list = []
xx = []
yy = []

#načtení informací z listu
for f in range(len(centerlist_lidar)):
    indexes.append(centerlist_lidar[f][1])
    xx.append(centerlist_lidar[f][2][0])
    yy.append(centerlist_lidar[f][2][1])

minimal_occurence = 4

#Největší index
limit = indexes[-1]

#Filtrování málo frekventovaných identifikátorů
for d in range(limit):
    s = indexes.count(d)
    #print("pocitani"+"vysledek "+str(s)+" prvek"+str(d))
    if s > minimal_occurence:
        right_detections.append(d)
    else:
        wrong_detections.append(d)

for k in range(len(centerlist_lidar)):
    if centerlist_lidar[k][1] in wrong_detections:
        pass
    else:
        correct_list.append(centerlist_lidar[k])

#print("right",len(right_detections))
#print(right_detections)
#print("wrong",len(wrong_detections))
#print(wrong_detections)

sumax = 0
sumay = 0
numsum = 0
total = []
totalx = []
totaly = []
lidtime = 0

#Průměrování souřadnic stromů se stejným identifikátorem
for h in range(len(right_detections)):
    for g in range(len(correct_list)):
        if right_detections[h] == correct_list[g][1]:
            sumax = sumax + correct_list[g][2][0]
            sumay = sumay + correct_list[g][2][1]
            lidtime = lidtime + lidart[correct_list[g][0]].total_seconds()
            numsum = numsum+1
        else:
            continue
    total.append([lidtime/numsum, [sumax / numsum, sumay / numsum]])
    #pro vizualizaci
    totalx.append(sumax / numsum)
    totaly.append(sumay / numsum)
    sumax = 0
    sumay = 0
    numsum = 0
    lidtime = 0

plt.figure(figsize=(6, 6))
plt.title("Lidarmap_clean")
plt.scatter(xx, yy, s=6, c='y',marker='o',label='lidarmap_raw')
plt.scatter(totalx, totaly, s=10, c='k', marker='s', label='lidarmap_clean')
plt.legend(loc='upper left')
plt.xlabel("x [mm]")
plt.ylabel("y [mm]")
plt.show()

joblib.dump(total,"data_memory/lidarmap_clean_"+str(res1)+"_"+str(res2)+".sav")

