This program was developed as a part of thesis for a project implementing Agriculture 4.0 principles. 
In this project, a garden robot equipped with various sensors drives arround a row of trees and scans it. Based on the scanning,
the data is processed and plotten onto a map. The resulting map is then supplemented with cropped images of individual trees for further analysis and processing.
The rest of the README.md explains program structure and used variable parameters in czech language primarily for usage by coworkers.

Program má stále strukturu z vývojové části pro kontrolu mezivýsledků.

SYSTEMATICKÝ POSTUP SPOUŠTĚNÍ SKRIPTŮ:

    stručný popis skripů + správné pořadí spouštění - vstupy a výstupy viz schéma.png 
    složka data_memory jednotlivé výstupy z jednotlivých částí programu - možná simulace každého zvlášť 

-data_gathering

    sběr dat, formátování, ukládání do listů

-data_merging

    časové sjednocení pose3d, gps a lidar dat - interpolace

-main fusion

    fuze informaci o pozici robota - gps + pose3d
    detekce stromů na základě center shluků v lidarových měřeních

-lidarmap_cleaner

    filtrování málo frekventovaných identifikátorů
    určení souřadnic stromů z lidarové mapy

-amap_maker

    tvorba apriorni mapy ze znamych gps souradnic sloupů a známého počtu stromů v poli

-camera_track

    tvorba mapy z kamerových dat
    detekce kmenů stromů na snímcích
    následné určení souřadnic stromů pomocí známé vzdálenosti stromové řady a vlastností kamery

-cameramap_cleaner

    filtrování málo frekventovaných identifikátorů
    určení souřadnic stromů z kamerové mapy

-map_fusion

    kombinace tří vstupních map na základě metody dva ze tří
    tvorba výsledné mapy

-photo_extractor
    využití známých souřadníc stromů pro zisk oříznutých snímků
    tvorba výsledné struktury tree_list
    
SHRNUTÍ A MOTIVACE ZA VÝVOJEM PROGRAMU
Cílem vývoje bylo využít dat ze senzorů zahradního robota pro tvorbu mapy sadu a extrakci fotografií jednotlivých stromů. Vstupem je soubor .log produkovaný systémem osgar. Aktuální verze programu využívá dat z října 2022. 

Pro implementaci na novych datech třeba zjistit strukturu logu a názvy kanálů: python3 -m osgar.logger NÁZEVLOGU.log

SEZNAM NASTAVITELNÝCH PARAMETRŮ:

main_fusion

	max_range = 1700
	min_range = 400
	angle_range_low = 315
	angle_range_high = 540
		- analýza pouze specifické oblasti z lidaru
		- vhodné zkontrolovat správnost určování shluků při změně
	
	centlowlim = -150
	centhighlim = 150
		- hledání shluků pouze v oblasti kolmo k robotovi
	minclusterpoints = 10
	maxclusterpoints = 120
		- min a max počty bodů v clusterech, pokud jsou mimo oblast, jsou zanedbávané
	
	ident_limit = 200
		- pro indetifikaci - maximální možný rozdíl mezi predikovanou pozicí posledního centra a skutečnou pozicí na novém snímku
		- zbytečně vysoké, většinou do pár cm.
	weight_pose3d = 0.1
	weight_gps = 0.9
		- váhy zdrojů informací o pozici pro fúzi
		
lidarmap_cleaner
	minimal_occurence = 4
		-pokud se strom nevyskytne minimálně 4x v listu je zanedbán
		
amap_maker
	num_poles =3
		-definice velikosti analyzovaného pole
		
camera_track
	distortion limit = 200
		-obdoba centerlowlim, centerhighlim - omezení se na střed snímku

map_fusion
	lidar_weight = 0.3
	camera_weight = 0.3
	amap_weight = 0.4
		-váhy pro fúzi
	merge_limit = 200
		-max vzdálenost pro zjištění odpovídajících stromů v jednotlivých listech
photo_extractor
	correction = 30
		- ručně zvoleno pro dosažení odpovídajícíh výsledků na jednotlivých kamerách, které jsou v reálu vyoseny, ideálně = 0
	default_crop
		- pokud YOLO nenajde strom, ořizne defaultní část
	nazev_stromu
		- zvolene pojmenovavaní vysledných fotografií
