import tkinter as tk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import joblib
from PIL import Image, ImageTk
import cv2
import PIL.Image, PIL.ImageTk
import math


dif_deg = 270 / 811
max_range = 5000
min_range = 400
angle_range_low = 0
angle_range_high = 810

#Synchronizace časů
route_sync = 0.669724771
rscolor_sync = 0.558103976
arecont_sync = 0.394495413

class App:
    def __init__(self, window, window_title, routecam, arecontcam, rscolorcam, gps_data, lidar_data, pose3d_data):
        self.window = window
        self.window.geometry("791x973")
        self.window.resizable(0, 0)
        self.window.title(window_title)
        self.routecam = routecam
        self.arecontcam = arecontcam
        self.rscolorcam = rscolorcam
        self.gps_data = gps_data
        self.pose3d_data = pose3d_data
        self.lidar_data = lidar_data
        self.paused=False

        self.globalindex = 0

        #Jednotlive graficke prvky
        self.route_canvas = tk.Canvas(window, width=791, height=420)
        self.route_canvas.pack()

        self.label_route = tk.Label(text="  Route CAM                Arecont CAM              RealSense CAM",
                                    height=1, width=150)
        self.label_route.config(font=("Calibri", 18))
        self.label_route.place(x=20, y=420)
        self.label_route.pack()

        self.fig, self.ax = plt.subplots(figsize=(5,5))
        self.ax.set_xlabel("x [mm]")
        self.ax.set_ylabel("y [mm]")
        self.ax.set_title("Lidar scan")
        self.ax.grid(visible=True)
        self.canvas = FigureCanvasTkAgg(self.fig, master=window)
        self.canvas.get_tk_widget().pack(side="left", padx=10)

        self.label_timer = tk.Label(height=1, width=25)
        self.label_timer.config(font=("Calibri", 16))
        self.label_timer.place(x=300, y=450)
        self.label_timer.pack(side="top", padx=5, pady=30)

        self.label_gps = tk.Label(text="GPS souřadnice: ", height=1, width=25)
        self.label_gps.config(font=("Calibri", 16))
        self.label_gps.place(x=650, y=450)
        self.label_gps.pack(side="top", padx=5, pady=15)

        self.label_gpsx = tk.Label(height=1, width=25)
        self.label_gpsx.config(font=("Calibri", 16))
        self.label_gpsx.place(x=650, y=450)
        self.label_gpsx.pack(side="top", padx=5)

        self.label_gpsy = tk.Label(height=1, width=25)
        self.label_gpsy.config(font=("Calibri", 16))
        self.label_gpsy.place(x=650, y=450)
        self.label_gpsy.pack(side="top", padx=5)

        self.label_pose = tk.Label(text="Pose3d souřadnice: ", height=1, width=25)
        self.label_pose.config(font=("Calibri", 16))
        self.label_pose.place(x=650, y=450)
        self.label_pose.pack(side="top", padx=5,pady=15)

        self.label_posex = tk.Label(height=1, width=25)
        self.label_posex.config(font=("Calibri", 16))
        self.label_posex.place(x=650, y=450)
        self.label_posex.pack(side="top", padx=5)

        self.label_posey = tk.Label(height=1, width=25)
        self.label_posey.config(font=("Calibri", 16))
        self.label_posey.place(x=650, y=450)
        self.label_posey.pack(side="top", padx=5)

        #self.label_poseyaw = tk.Label(height=1, width=25)
        #self.label_poseyaw.config(font=("Calibri", 16))
        #self.label_poseyaw.place(x=650, y=450)
        #self.label_poseyaw.pack(side="top", padx=5)

        self.label_range = tk.Label(text="Vzdálenost od řady: ", height=1, width=25)
        self.label_range.config(font=("Calibri", 16))
        self.label_range.place(x=650, y=450)
        self.label_range.pack(side="top", padx=5, pady=15)

        self.label_rangeleft = tk.Label(height=1, width=25)
        self.label_rangeleft.config(font=("Calibri", 16), fg="red")
        self.label_rangeleft.place(x=650, y=450)
        self.label_rangeleft.pack(side="top", padx=5)

        self.label_rangeright = tk.Label(height=1, width=25)
        self.label_rangeright.config(font=("Calibri", 16), fg="green")
        self.label_rangeright.place(x=650, y=450)
        self.label_rangeright.pack(side="top", padx=5)

        #Automaticke obnoveni
        self.delay = 1
        self.update()
        self.window.mainloop()


    def update(self):
        #routecam
        routecam_picture_crop = cv2.resize(self.routecam[round(self.globalindex*route_sync)][1], (400, 225))
        routecam_picture_crop = cv2.rotate(routecam_picture_crop, cv2.ROTATE_90_CLOCKWISE)
        cv2.imwrite("routecam_picture_crop.png", routecam_picture_crop)
        cv2.waitKey(1)
        #Tento přístup využivá PIL.Image.open, který otevírá vytvořený obraz, v každém cyklu je tedy třeba obraz
        #nejdříve načíst, uložit a pak opět otevřit. Alternativou této funkce je PIL.Image.from_array, která načítá
        #obraz přímo z numpy pole. Bohužel se při tomto převodu ztratí informace o barvě a jablka jsou pak modrá.
        argroute = PIL.Image.open("routecam_picture_crop.png")
        self.photo = PIL.ImageTk.PhotoImage(argroute)
        self.route_canvas.create_image(10, 10, image=self.photo, anchor=tk.NW)

        #arecontcam
        arecontcam_picture_crop = cv2.resize(self.arecontcam[round(self.globalindex*arecont_sync)][1], (400, 300))
        arecontcam_picture_crop = cv2.rotate(arecontcam_picture_crop, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imwrite("arecontcam_picture_crop.png", arecontcam_picture_crop)
        cv2.waitKey(1)
        argarecont = PIL.Image.open("arecontcam_picture_crop.png")
        self.photo2 = PIL.ImageTk.PhotoImage(image=argarecont)
        self.route_canvas.create_image(245, 10, image=self.photo2, anchor=tk.NW)

        #rscolorcam
        rscolorcam_picture_crop = cv2.resize(self.rscolorcam[round(self.globalindex*rscolor_sync)][1], (400, 226))
        rscolorcam_picture_crop = cv2.rotate(rscolorcam_picture_crop, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imwrite("rscolorcam_picture_crop.png", rscolorcam_picture_crop)
        cv2.waitKey(1)
        argrscolor = PIL.Image.open("rscolorcam_picture_crop.png")
        self.photo3 = PIL.ImageTk.PhotoImage(image=argrscolor)
        self.route_canvas.create_image(555, 10, image=self.photo3, anchor=tk.NW)


        self.gpsx = self.gps_data[self.globalindex][1][0]
        self.gpsy = self.gps_data[self.globalindex][1][1]
        self.posex=self.pose3d_data[self.globalindex][1][0]
        self.posey=self.pose3d_data[self.globalindex][1][1]
        #self.poseyaw=self.pose3d_data[self.globalindex][2]
        self.timer=self.gps_data[self.globalindex][0].total_seconds()

        #lidar
        self.ax.clear()
        merged_lidar_short_xy = []
        lidar_left = []
        lidar_right = []
        A = self.lidar_data[self.globalindex][1]
        #Odlišné oproti main_fusion, protože je zde lidar natočený jiným směrem.
        for l in range(angle_range_low, angle_range_high, 1):
            if min_range < A[l] < max_range:
                if (-45 <= -45 + dif_deg * l) & (-45 + dif_deg * l < 0):
                    merged_lidar_short_xy.append([+A[l] * math.cos(math.radians(45 - dif_deg * l)),
                                                  -A[l] * math.sin(math.radians(45 - dif_deg * l))])
                    lidar_right.append([+A[l] * math.cos(math.radians(45 - dif_deg * l)),
                                        -A[l] * math.sin(math.radians(45 - dif_deg * l))])
                elif (0 < -45 + dif_deg * l) & (-45 + dif_deg * l < 90):
                    merged_lidar_short_xy.append([+A[l] * math.cos(math.radians(-45 + dif_deg * l)),
                                                  +A[l] * math.sin(math.radians(-45 + dif_deg * l))])
                    lidar_right.append([+A[l] * math.cos(math.radians(-45 + dif_deg * l)),
                                        +A[l] * math.sin(math.radians(-45 + dif_deg * l))])
                elif (90 < -45 + dif_deg * l) & (-45 + dif_deg * l < 180):
                    merged_lidar_short_xy.append([-A[l] * math.sin(math.radians(-135 + dif_deg * l)),
                                                  +A[l] * math.cos(math.radians(-135 + dif_deg * l))])
                    lidar_left.append([-A[l] * math.sin(math.radians(-135 + dif_deg * l)),
                                       +A[l] * math.cos(math.radians(-135 + dif_deg * l))])
                elif (180 < -45 + dif_deg * l) & (-45 + dif_deg * l <= 225):
                    merged_lidar_short_xy.append([-A[l] * math.cos(math.radians(-225 + dif_deg * l)),
                                                  -A[l] * math.sin(math.radians(-225 + dif_deg * l))])
                    lidar_left.append([-A[l] * math.cos(math.radians(-225 + dif_deg * l)),
                                       -A[l] * math.sin(math.radians(-225 + dif_deg * l))])
                else:
                    print("Out of lidar FOV")
            else:
                continue
        x_right, y_right = zip(*lidar_right)
        x_left, y_left = zip(*lidar_left)

        #vzdaleny bod rady
        self.farnearR, self.farnear_distR = self.find_nearest_point(target=[1200, 5000], list=lidar_right)
        self.farnearL, self.farnear_distL = self.find_nearest_point(target=[-1000, 5000], list=lidar_left)
        #blizky bod rady
        self.nearR, self.near_distR = self.find_nearest_point(target=[0, 0], list=lidar_right)
        self.nearL, self.near_distL = self.find_nearest_point(target=[0, 0], list=lidar_left)

        #referencni stredova linie
        self.middleline = [(self.nearR[0]+self.nearL[0])/2, (self.nearR[1]+self.nearL[1])/2]
        self.middlelinefar = [(self.farnearR[0]+self.farnearL[0])/2, (self.farnearR[1]+self.farnearL[1])/2]
        self.a = (self.middlelinefar[1]-self.middleline[1])/(self.middlelinefar[0]-self.middleline[0])
        self.deviation_angle = math.degrees(math.atan((self.middlelinefar[0]-self.middleline[0]) /
                                                      (self.middlelinefar[1]-self.middleline[0])))
        self.b = self.middleline[1] - self.a * self.middleline[0]
        #vypocet kolmice a pruseciku s bodem [0,0]
        self.b_k = 0
        self.a_k = -1/self.a
        self.x_intersect = (self.b_k - self.b) / (self.a - self.a_k)
        self.y_intersect = self.a * self.x_intersect + self.b

        #vzdalenost od stredove linie v kolmem smeru
        self.shortest_dist = math.sqrt(math.pow(self.x_intersect, 2) + math.pow(self.y_intersect, 2))
        if self.x_intersect >= 0:
            self.shortest_dist = -self.shortest_dist
        else:
            self.shortest_dist = self.shortest_dist

        #update jednotlivych textovych bloku
        self.label_timer.config(text="Timer: " + str(round(self.timer, 4)) + " s")
        self.label_gpsx.config(text="E: " + str(round(self.gpsx, 10)) + "°")
        self.label_gpsy.config(text="N: " + str(round(self.gpsy, 10)) + "°")
        self.label_posex.config(text="x: " + str(round(self.posex, 4)) + " mm")
        self.label_posey.config(text="y: " + str(round(self.posey, 4)) + " mm")
        #self.label_poseyaw.config(text="úhel: " + str(round(self.poseyaw,4)) + "°")
        self.label_rangeright.config(text="R: " + str(round(self.near_distR, 4)) + " mm")
        self.label_rangeleft.config(text="L: " + str(round(self.near_distL, 4)) + " mm")

        #vykresleni prvku grafu
        self.ax.scatter(x_right, y_right, marker='o', c='g')
        self.ax.scatter(x_left, y_left, marker='o', c='r')
        xmin, xmax, ymin, ymax = self.ax.axis([-2500, 2500, -1500, 5000])

        self.ax.plot([0, self.nearR[0]], [0, self.nearR[1]], c='k', linestyle='dashed')
        self.ax.plot([0, self.nearL[0]], [0, self.nearL[1]], c='k', linestyle='dashed')
        self.ax.plot([self.nearR[0], self.farnearR[0]], [self.nearR[1], self.farnearR[1]], c='b', linestyle='dashed')
        self.ax.plot([self.nearL[0], self.farnearL[0]], [self.nearL[1], self.farnearL[1]], c='b', linestyle='dashed')
        self.ax.plot([self.middleline[0], self.middlelinefar[0]], [self.middleline[1], self.middlelinefar[1]], c='m', linestyle='dashed')
        self.ax.grid(visible=True)
        self.ax.set_xlabel("x [mm]")
        self.ax.set_ylabel("y [mm]")
        self.ax.set_title("Lidar scan")
        self.canvas.draw()

        self.globalindex += 1
        #print(self.globalindex)
        self.window.after(self.delay, self.update)

#nalezeni nejblizsiho bodu v listu
    def find_nearest_point(self,target, list):
        nearest_point = list[0]
        min_dist = math.dist(target, nearest_point)
        for point in list[1:]:
            dist = math.dist(target, point)
            if dist < min_dist:
                 min_dist = dist
                 nearest_point = point
        return nearest_point, min_dist

# urceni vzdalenosti jako kolme vzdalenosti od primky pomoci linearni regrese
"""
    def get_distance(self,lidar_list, expected_point_x, expected_point_y):
        nearest_point, min_dist =self.find_nearest_point(target=[expected_point_x,expected_point_y],list=lidar_list)
        if min_dist > 500:
        else:
            lidarxx, lidaryy = zip(*lidar_list)
            lidarxx=list(lidarxx)
            lidaryy=list(lidaryy)
           
            fx=np.mean(lidarxx)
            lidarx=[]
            lidary=[]
            lidart=[]
            for ii in range(len(lidarxx)):
                if abs(lidarxx[ii] - fx) < 700:
                    lidarx.append(lidarxx[ii])
                    lidary.append(lidaryy[ii])
                    lidart.append([lidarxx[ii],lidaryy[ii]])

            centerx, centery = zip(*lidart)
            xx = np.array(centerx).reshape(-1, 1)
            yy = np.array(centery).reshape(-1, 1)
            linear_regressor = LinearRegression()
            linear_regressor.fit(xx, yy)
            Y_pred = linear_regressor.predict(xx)
            a = linear_regressor.coef_[0][0]
            b = nearest_point[1]- a * nearest_point[0]
            #kolmice
            a_k = -1/a
            b_k = 0
            x_intersect = (b_k - b)/(a - a_k)
            y_intersect = a * x_intersect + b
            ranged = math.sqrt(math.pow(x_intersect,2)+math.pow(y_intersect,2))
            return Y_pred, xx , ranged, yy
"""
#spusteni aplikace a nacteni vstupu
App(window=tk.Tk(),
    window_title="Kontrola prubehu mereni",
    routecam=joblib.load('data_memory_new/routecam_data_full_29.5_102.5.sav'),
    rscolorcam=joblib.load('data_memory_new/rscolor_data_full_29.5_102.5.sav'),
    arecontcam=joblib.load('data_memory_new/arecontcam_data_full_29.5_102.5.sav'),
    gps_data=joblib.load('data_memory_new/gps_data_29.5_102.5.sav'),
    pose3d_data=joblib.load('data_memory_new/pose3d_data_29.5_102.5.sav'),
    lidar_data=joblib.load('data_memory_new/lidar_data_29.5_102.5.sav')
    )
