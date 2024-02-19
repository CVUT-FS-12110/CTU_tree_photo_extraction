import joblib
import math
import numpy as np
from geopy.distance import distance
import cv2
from ultralytics import YOLO
import torch

res1 = 31
res2 = 112.7
#res2 = 546

gps_data = joblib.load('data_memory/gps_data_'+str(res1)+'_'+str(res2)+'.sav')
fused_map = joblib.load('data_memory/fused_map_'+str(res1)+'_'+str(res2)+'.sav')
fused_trajectory = joblib.load('data_memory/fused_trajectory_'+str(res1)+'_'+str(res2)+'.sav')
rscolor = joblib.load('data_memory/rscolor_data_full_'+str(res1)+'_'+str(res2)+'.sav')
arecontcam = joblib.load('data_memory/arecontcam_data_full_'+str(res1)+'_'+str(res2)+'.sav')
missing_trees = joblib.load('data_memory/missing_trees_'+str(res1)+"_"+str(res2)+'.sav')

torch.cuda.set_device(0)
model = YOLO("data_memory/yolov8.pt")
start_gps = [gps_data[0][1][0], gps_data[0][1][1]]

#Konstanty a nastavení
w_arecont, h_arecont = arecontcam[0][1].shape[:2]
w_rscolor, h_rscolor, = rscolor[0][1].shape[:2]
halfcrop_arecont = 220
halfcrop_rscolor = 125
distortion_limit = 200
correction = 30 #v realite je rs kamera namontovana trochu nakrivo a neni v ose s arecontkou
default_crop_arecont = [[0, h_arecont], [230, 230+2*halfcrop_arecont]]
default_crop_rscolor = [[0, h_rscolor], [135, 135+2*halfcrop_rscolor]]

fusedx = []
fusedy = []
fusedtime = []
fused_position = []
tree_list = []

#Seřazení stromů do správného pořadí
for nn in range(len(fused_map)):
    fusedx.append(fused_map[nn][1][0])
    fusedy.append(fused_map[nn][1][1])
    fusedtime.append(fused_map[nn][0])
    fused_position.append([fused_map[nn][1][0], fused_map[nn][1][1]])

fused_position = np.array(fused_position)
fusedtime = np.array(fusedtime)
idx = np.argsort(fusedtime)
fused_position = np.array(fused_position)[idx].tolist()
fusedtime = np.array(fusedtime)[idx].tolist()


#gap_index
def biggest_gaps(seznam, pocet):
    # Zkontrolujte, zda je seznam dostatečně dlouhý pro vyhledání mezer
    if len(seznam) < 2 or pocet <= 0:
        return "Neplatný vstup"

    # Vytvořte seznam mezer mezi sousedními prvky a jejich indexů
    mezery_s_indexy = [(abs(seznam[i] - seznam[i + 1]), i) for i in range(len(seznam) - 1)]

    # Seřaďte seznam mezer sestupně
    serazene_mezery_s_indexy = sorted(mezery_s_indexy, key=lambda x: x[0], reverse=True)

    # Získání indexů požadovaného počtu největších mezer
    indexy_nejvetsich_mezer = [index for (mezera, index) in serazene_mezery_s_indexy[:pocet]]
    indexy_nejvetsich_mezer = list(map(lambda x: x + 2, indexy_nejvetsich_mezer))

    return indexy_nejvetsich_mezer


missing_indexes = biggest_gaps(fusedtime, missing_trees)
rscolortimer = []
for uu in range(len(rscolor)):
    rscolortimer.append(rscolor[uu][0].total_seconds())

areconttimer = []
for ii in range(len(arecontcam)):
    areconttimer.append(arecontcam[ii][0].total_seconds())

def a2rs(xbox, w_rscolor, w_arecont):
    x_conv = xbox*w_rscolor/w_arecont
    return x_conv


def find_nearest(array, value):
    #vraci index nejblizsi vyssi hodnoty
    idx = np.searchsorted(array, value, side="left")
    if idx > 0 and (idx == len(array) or (value - array[idx-1]) < (value - array[idx])):
        return idx-1
    else:
        return idx


#Neuronová síť detekuje správně pouze na jednom druhu kamery
# => Najdeme kmen na snímkách pouze z jednoho druhu kamery a následně díky známému vztahu mezi kamerami převedeme informaci pro oriznuti.
cislostromu = 1
for s in range(len(fusedtime)):
    arecont_ind = find_nearest(areconttimer, fusedtime[s])
    rscolor_ind = find_nearest(rscolortimer, fusedtime[s])

    frame_arecont = arecontcam[arecont_ind][1]
    frame_rscolor = rscolor[rscolor_ind][1]

    frame_rscolor = cv2.rotate(frame_rscolor, cv2.ROTATE_90_COUNTERCLOCKWISE)
    frame_arecont = cv2.rotate(frame_arecont, cv2.ROTATE_90_CLOCKWISE)

    result_arecont = model.track(frame_arecont, persist=True, conf=0.2, iou=0.5)
    boxes = result_arecont[0].boxes.xyxy

    fitcrop = False
    fitm = -9
    if boxes is None:
        # Na snimku nebyl detekovan zadny strom - vychozi interval orezu
        frame_arecont = frame_arecont[:, default_crop_arecont[1][0]
                                         :default_crop_arecont[1][1]]
        frame_rscolor = frame_rscolor[:, default_crop_rscolor[1][0]
                                         :default_crop_rscolor[1][1]]
    else:
        # Kontrola detekovanych boxu
        for m in range(len(boxes)):
            xbox = float(boxes[m][0] + boxes[m][2]) / 2
            # Vybirame si strom blizko stredu
            if xbox > distortion_limit and xbox < w_arecont - distortion_limit:
                fitcrop = True
                fitm = m
        if fitcrop:
            xbox = float(boxes[fitm][0] + boxes[fitm][2]) / 2
            # Byl nalezen strom blizko stredu - prizpusobeni intervalu orezu
            frame_arecont = frame_arecont[:, int(xbox) - halfcrop_arecont + 1
                                             :int(xbox) + halfcrop_arecont]
            frame_rscolor = frame_rscolor[:, int(a2rs(xbox, w_rscolor, w_arecont)) -
                                             halfcrop_rscolor + correction + 1:
                                             int(a2rs(xbox, w_rscolor, w_arecont)) +
                                             halfcrop_rscolor - correction]
        else:
            # Nebyl detekovan strom blizko stredu - vychozi interval orezu
            frame_arecont = frame_arecont[:, default_crop_arecont[1][0]
                                             :default_crop_arecont[1][1]]
            frame_rscolor = frame_rscolor[:, default_crop_rscolor[1][0]
                                             :default_crop_rscolor[1][1]]

    annotated_frame_arecont = result_arecont[0].plot()
    cv2.imshow("Tracking Arecont", annotated_frame_arecont)
    cv2.waitKey(1)

    #Kontrola chybějících stromů
    if cislostromu in missing_indexes:
        cislostromu += 1
    #Dosažení požadovaného formátu číslování
    c = cislostromu % 20
    pole = math.floor(cislostromu/20)+1
    if (cislostromu % 20) == 0:
        pole = pole-1
        c = 20

    nazevstromu = "4L_"+str(pole)+"_"+str(c)
    #cv2.imwrite("data_memory/photos/arecont/strom_"+nazevstromu+"_"+str(areconttimer[arecont_ind])+'.jpg',frame_arecont)
    #cv2.imwrite("data_memory/photos/rscolor/strom_"+nazevstromu+"_"+str(rscolortimer[rscolor_ind])+'.jpg',frame_rscolor)
    cislostromu += 1

    #Převod x,y na GPS
    gpsmove_x = (distance(meters=fused_position[s][0] / 1000).
                 destination((start_gps[1], start_gps[0]), bearing=90))
    gpsmove_y = (distance(meters=fused_position[s][1] / 1000).
                 destination((gpsmove_x.latitude, gpsmove_x.longitude),bearing=0))

    gps_position = [gpsmove_y.longitude, gpsmove_y.latitude]
    # [nazev stromu, gps, arecont snimek, realsense snimek, xy pozice, cas mereni]
    tree = [str(nazevstromu), gps_position, frame_arecont, frame_rscolor, fused_position[s], fusedtime[s]]
    tree_list.append(tree)

#joblib.dump(tree_list, 'data_memory/tree_list_'+(str(res1)+'_'+str(res2)+'.sav'))


