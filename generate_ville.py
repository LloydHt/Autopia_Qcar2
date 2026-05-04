import cv2
import numpy as np

# --- PARAMÈTRES DE BASE ---
size = 10000 # 20m x 20m (Échelle : 1px = 2mm)
w = 300      # Demi-largeur de la route (Route = 600px = 1.2m réels)
yellow_w = 12 # Épaisseur de la ligne jaune

# Couleurs
C_HERBE = (45, 130, 45)       
C_ROUTE = (60, 60, 60)        
C_TROTTOIR = (180, 180, 180)  
C_BATIMENT = (140, 150, 160)  
C_LIGNE = (0, 255, 255)       
C_BLANC = (255, 255, 255)     

img = np.full((size, size, 3), C_HERBE, dtype=np.uint8)

# Positions clés du quadrillage
p_left = 2500
p_right = 7500
p_top = 2500
p_bottom = 7500
p_center = 5000

# --- 1. DESSINER LES QUARTIERS ---
def draw_neighborhood_block(x1, y1, x2, y2):
    cv2.rectangle(img, (x1, y1), (x2, y2), C_TROTTOIR, -1)
    m = 200 
    cv2.rectangle(img, (x1+m, y1+m), (x2-m, y2-m), C_BATIMENT, -1)
    cv2.rectangle(img, (x1+m, y1+m), (x2-m, y2-m), (100, 100, 100), 10)

draw_neighborhood_block(p_left+w, p_top+w, p_center-w, p_center-w) 
draw_neighborhood_block(p_center+w, p_top+w, p_right-w, p_center-w) 
draw_neighborhood_block(p_left+w, p_center+w, p_center-w, p_bottom-w) 
draw_neighborhood_block(p_center+w, p_center+w, p_right-w, p_bottom-w) 

# --- 2. DESSINER LES ROUTES (Bitume) ---
# Anneau carré extérieur
cv2.rectangle(img, (p_left-w, p_top-w), (p_right+w, p_top+w), C_ROUTE, -1) 
cv2.rectangle(img, (p_left-w, p_bottom-w), (p_right+w, p_bottom+w), C_ROUTE, -1) 
cv2.rectangle(img, (p_left-w, p_top-w), (p_left+w, p_bottom+w), C_ROUTE, -1) 
cv2.rectangle(img, (p_right-w, p_top-w), (p_right+w, p_bottom+w), C_ROUTE, -1) 

# Croix intérieure
cv2.rectangle(img, (p_left-w, p_center-w), (p_right+w, p_center+w), C_ROUTE, -1) 
cv2.rectangle(img, (p_center-w, p_top-w), (p_center+w, p_bottom+w), C_ROUTE, -1) 

# 🌟 NOUVEAU : Les 4 Sorties (Nord, Sud, Est, Ouest)
cv2.rectangle(img, (p_center-w, 0), (p_center+w, p_top-w), C_ROUTE, -1)        # Nord
cv2.rectangle(img, (p_center-w, p_bottom+w), (p_center+w, size), C_ROUTE, -1)  # Sud
cv2.rectangle(img, (0, p_center-w), (p_left-w, p_center+w), C_ROUTE, -1)       # Ouest
cv2.rectangle(img, (p_right+w, p_center-w), (size, p_center+w), C_ROUTE, -1)   # Est

# --- 3. LE ROND-POINT CENTRAL ---
cv2.circle(img, (p_center, p_center), 800, C_ROUTE, -1)     
cv2.circle(img, (p_center, p_center), 415, C_BLANC, 10)     
cv2.circle(img, (p_center, p_center), 400, C_HERBE, -1)     

# --- 4. PASSAGES PIÉTONS ---
def draw_crosswalk(x, y, crossing_horizontal_road=True):
    strip_w = 40  
    strip_h = 200 
    gap = 80      
    for i in range(-3, 4): 
        cx_bande = x + i * gap
        cy_bande = y + i * gap
        if crossing_horizontal_road:
            cv2.rectangle(img, (cx_bande - strip_w//2, y - strip_h), (cx_bande + strip_w//2, y + strip_h), C_BLANC, -1)
        else:
            cv2.rectangle(img, (x - strip_h, cy_bande - strip_w//2), (x + strip_h, cy_bande + strip_w//2), C_BLANC, -1)

# A. 🌟 NOUVEAU : Passages piétons sur les 4 Sorties extérieures
draw_crosswalk(p_center, p_top - w - 250, crossing_horizontal_road=True)       # Nord
draw_crosswalk(p_center, p_bottom + w + 250, crossing_horizontal_road=True)    # Sud
draw_crosswalk(p_left - w - 250, p_center, crossing_horizontal_road=False)     # Ouest
draw_crosswalk(p_right + w + 250, p_center, crossing_horizontal_road=False)    # Est

# B. Entrées du rond-point
draw_crosswalk(p_center, p_center - 1100, crossing_horizontal_road=True) # Nord
draw_crosswalk(p_center, p_center + 1100, crossing_horizontal_road=True) # Sud
draw_crosswalk(p_center - 1100, p_center, crossing_horizontal_road=False) # Ouest
draw_crosswalk(p_center + 1100, p_center, crossing_horizontal_road=False) # Est

# C. Passages Coins (Herbe <-> Bâtiments)
draw_crosswalk(p_left + 700, p_top, crossing_horizontal_road=True); draw_crosswalk(p_left, p_top + 700, crossing_horizontal_road=False)
draw_crosswalk(p_right - 700, p_top, crossing_horizontal_road=True); draw_crosswalk(p_right, p_top + 700, crossing_horizontal_road=False)
draw_crosswalk(p_left + 700, p_bottom, crossing_horizontal_road=True); draw_crosswalk(p_left, p_bottom - 700, crossing_horizontal_road=False)
draw_crosswalk(p_right - 700, p_bottom, crossing_horizontal_road=True); draw_crosswalk(p_right, p_bottom - 700, crossing_horizontal_road=False)

# --- 5. LIGNES JAUNES POINTILLÉES ---
def draw_dashed_yellow(x1, y1, x2, y2):
    dist = int(np.sqrt((x2-x1)**2 + (y2-y1)**2))
    dash_len = 100; gap_len = 100
    for i in range(0, dist, dash_len + gap_len):
        if i + dash_len < dist:
            start = (int(x1 + (x2-x1)*(i/dist)), int(y1 + (y2-y1)*(i/dist)))
            end = (int(x1 + (x2-x1)*((i+dash_len)/dist)), int(y1 + (y2-y1)*((i+dash_len)/dist)))
            cv2.line(img, start, end, C_LIGNE, yellow_w)

# Anneau extérieur
offset_corner = 950 
draw_dashed_yellow(p_left + offset_corner, p_top, p_right - offset_corner, p_top) 
draw_dashed_yellow(p_left + offset_corner, p_bottom, p_right - offset_corner, p_bottom) 
draw_dashed_yellow(p_left, p_top + offset_corner, p_left, p_bottom - offset_corner) 
draw_dashed_yellow(p_right, p_top + offset_corner, p_right, p_bottom - offset_corner) 

# Croix intérieure
offset_roundabout = 1300 
draw_dashed_yellow(p_left, p_center, p_center - offset_roundabout, p_center) 
draw_dashed_yellow(p_center + offset_roundabout, p_center, p_right, p_center) 
draw_dashed_yellow(p_center, p_top, p_center, p_center - offset_roundabout) 
draw_dashed_yellow(p_center, p_center + offset_roundabout, p_center, p_bottom) 

# 🌟 NOUVEAU : Les 4 Sorties vers l'extérieur
offset_exit = 450 
draw_dashed_yellow(p_center, 0, p_center, p_top - w - offset_exit)             # Nord
draw_dashed_yellow(p_center, p_bottom + w + offset_exit, p_center, size)       # Sud
draw_dashed_yellow(0, p_center, p_left - w - offset_exit, p_center)            # Ouest
draw_dashed_yellow(p_right + w + offset_exit, p_center, size, p_center)        # Est

# --- FIN ET EXPORT ---
filename = 'autopia_ground.png'
cv2.imwrite(filename, img)
print(f"Succès ! L'image '{filename}' avec 4 entrées/sorties a été générée.")