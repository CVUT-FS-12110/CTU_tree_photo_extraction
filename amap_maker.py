import joblib
import math
from matplotlib import pyplot as plt
from geopy.distance import distance

res1 = 31
res2 = 112.7
#res2 = 546

gps_data = joblib.load('data_memory/gps_data_'+str(res1)+'_'+str(res2)+'.sav')

sloupy_gps=[[15.569373884238743, 50.37002728347285],
            [15.569406805671642, 50.37011603134719],
            [15.569438702474857, 50.370204689156736],
            [15.569470587705158, 50.370292157394715],
            [15.569502159324518, 50.37037936728204],
            [15.569532182312187, 50.37046651830891],
            [15.569564555285694, 50.37055472744731],
            [15.569596103861993, 50.37064194715937],
            [15.569627301174608, 50.37072933599747],
            [15.569658971588849, 50.37081723596066],
            [15.569690009118643, 50.370904477443304],
            [15.569721377293689, 50.37099287375822],
            [15.56975212092143, 50.37108018176467],
            [15.569780411306475, 50.37116000418904]]

start_gps = [gps_data[0][1][0], gps_data[0][1][1]]
num_poles = 3

#Funkce pro prevod gps na xy
def gps2xy(sloupgps, startgps):
    sloupx, sloupy = sloupgps
    startx, starty = startgps
    coords = [distance([starty, startx], [starty, sloupx]).m * 1000,
              distance([starty, startx], [sloupy, startx]).m * 1000]

    return coords

#Generování stromů
def generate_trees(point_1, point_2, num_trees):
    x1, y1 = point_1
    x2, y2 = point_2
    pole_dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    #Vzdalenost mezi jednotlivymi stromy
    spacing = pole_dist / num_trees
    print("spacing", spacing)
    unit_vector = ((x2 - x1)/pole_dist, (y2 - y1)/pole_dist)
    tree_coordinates = []
    for i in range(num_trees):
        #Vypocet pozice kazdeho individualniho stromu
        x_tree = x2 - (i+0.5) * spacing * unit_vector[0]
        y_tree = y2 - (i+0.5) * spacing * unit_vector[1]
        tree_coordinates.append([x_tree, y_tree])
    return tree_coordinates


stromy=[]
sloupy_xy=[]
#Převod GPS na xy a generování stromů
for h in range(num_poles):
    sloup_conv = gps2xy(sloupy_gps[h], start_gps)
    sloupy_xy.append(sloup_conv)
    if h == 13:
        strom = generate_trees(sloupy_xy[h-1],sloupy_xy[h],18)
        stromy = stromy + strom
    elif h>0:
        strom = generate_trees(sloupy_xy[h-1],sloupy_xy[h],20)
        stromy = stromy + strom


#Vizualizace
polesx, polesy = zip(*sloupy_xy)
treesx, treesy = zip(*stromy)
plt.figure(figsize=(6, 6))
plt.title("Apriorní mapa")
plt.xlabel("x [mm]")
plt.ylabel("y [mm]")
plt.scatter(treesx, treesy, s=10, c='k', marker='o', label='Stromy')
plt.scatter(polesx, polesy, s=10, c='m', marker='o', label='Sloupy')
plt.legend(loc='upper left',fontsize='large')

plt.show()

#Uložení
#joblib.dump(stromy, 'data_memory/amap_trees_'+str(res1)+'_'+str(res2)+'.sav')
#joblib.dump(sloupy_xy, 'data_memory/amap_poles_'+str(res1)+'_'+str(res2)+'.sav')
