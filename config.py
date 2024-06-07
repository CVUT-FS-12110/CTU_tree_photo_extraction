# konfigurační proměnné pro všechny soubory

# cesta k logu, který má být analyzován
log_file = 'data/ipc-dynamic-meas-240419_210554.log'

# soubor ipc-dynamic-meas-240419_210554.log
# 0:0:27:447747 start - prvni strom za sloupem (0)
# 0:0:55:150641 prvni strom za dalsim sloupem (1)
# 0:1:22:453993 sloup 2
# 0:1:23:318242 prvni strom za dalsim sloupem (2)
# 0:1:48:535941 sloup 3
# 0:1:49:266865 prvni strom za dalsim sloupem (3)

# 0:3:33:404827 posledni strom (fyzicky chybí, je tam jen tyčka)
# 0:3:34:911915 prvni strom za dalsim sloupem (3)

#Zvolené časové rozmezi z logu v sekundách
res1 = 27.447747
res2 = 109.266865

start_pole = 7# počáteční sloup ve zpracovávaném úseku
num_poles = 3 # počet sloupů ve zpracovávaném úseku (polí je o jedno méně)


