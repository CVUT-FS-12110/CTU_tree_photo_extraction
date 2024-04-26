import joblib
import numpy as np

res1 = 31
res2 = 112.7
#res2 = 546
gps_data_full = joblib.load('data_memory/gps_data_full_'+str(res1)+'_'+str(res2)+'.sav')
pose2d_data_full = joblib.load('data_memory/pose2d_data_full_'+str(res1)+'_'+str(res2)+'.sav')
lidar_data_full = joblib.load('data_memory/lidar_data_full_'+str(res1)+'_'+str(res2)+'.sav')

lidar_timestamps = []
lidar_timestamps_td = []
gps_timestamps = []
pose2d_timestamps = []
gps_data = []
pose2d_data = []

#Uložení časů jednotlivých datových záznamů
for l in range(len(lidar_data_full)):
    lidar_timestamps.append((lidar_data_full[l][0].total_seconds()))
    lidar_timestamps_td.append(lidar_data_full[l][0])

for i in range(len(gps_data_full)):
    gps_timestamps.append((gps_data_full[i][0].total_seconds()))

for k in range(len(pose2d_data_full)):
    pose2d_timestamps.append((pose2d_data_full[k][0].total_seconds()))
    #korekce pouze posunutim

#Funkce vracející index nejbližší hodnoty
def find_nearest(array, value):
    idx = np.searchsorted(array, value, side="left")
    if idx > 0 and (idx == len(array) or (value - array[idx-1]) < (value - array[idx])):
        return idx-1
    else:
        return idx

for m in range(len(lidar_timestamps)):
    interpol_gps = []
    interpol_pose2d = []

    interpol_pose2d.clear()
    interpol_gps.clear()

    gps_ind = find_nearest(array=gps_timestamps, value=lidar_timestamps[m])
    pose2d_ind = find_nearest(array=pose2d_timestamps, value=lidar_timestamps[m])

    for p in range(len(gps_data_full[1][1])):
        interpol_gps_1 = (gps_data_full[gps_ind-1][1][p] + (lidar_timestamps[m]-gps_timestamps[gps_ind-1]) *
                          ((gps_data_full[gps_ind][1][p]-gps_data_full[gps_ind-1][1][p]) /
                           (gps_timestamps[gps_ind]-gps_timestamps[gps_ind-1])))
        interpol_gps.append(interpol_gps_1)

    for o in range(len(pose2d_data_full[1][1])):
        interpol_pose2d_1 = (pose2d_data_full[pose2d_ind-1][1][o] + (lidar_timestamps[m]-pose2d_timestamps[pose2d_ind-1]) *
                             ((pose2d_data_full[pose2d_ind][1][o]-pose2d_data_full[pose2d_ind-1][1][o]) /
                              (pose2d_timestamps[pose2d_ind]-pose2d_timestamps[pose2d_ind-1])))
        interpol_pose2d.append(interpol_pose2d_1)

    gps_data.append([lidar_timestamps_td[m], [interpol_gps[0], interpol_gps[1]]])
    pose2d_data.append([lidar_timestamps_td[m], [interpol_pose2d[0], interpol_pose2d[1]], interpol_pose2d[2]])

print(pose2d_data)
print(gps_data)
print("gps", len(gps_data))
print("pose2d", len(pose2d_data))
print("lidar", len(lidar_data_full))

#Ukládací část, pro následující používání programu nutné odkomentovat.
joblib.dump(gps_data, 'data_memory/gps_data_'+str(res1)+'_'+str(res2)+'.sav')
joblib.dump(pose2d_data, 'data_memory/pose2d_data_'+str(res1)+'_'+str(res2)+'.sav')
joblib.dump(lidar_data_full, 'data_memory/lidar_data_'+str(res1)+'_'+str(res2)+'.sav')


