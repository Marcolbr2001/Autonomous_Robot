import cv2
import numpy as np

# Carica l'immagine da testare
image = cv2.imread("images/frame.png")  # Sostituisci con la tua immagine
if image is None:
    raise Exception("Immagine non trovata. Assicurati che 'frame.png' sia nel path corretto.")

cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Trackbars", 600, 400)

# Callback vuoto
def nothing(x):
    pass

# === TRACKBAR GIALLO ===
cv2.createTrackbar("Y_H_min", "Trackbars", 20, 179, nothing)
cv2.createTrackbar("Y_H_max", "Trackbars", 28, 179, nothing)
cv2.createTrackbar("Y_S_min", "Trackbars", 195, 255, nothing)
cv2.createTrackbar("Y_S_max", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("Y_V_min", "Trackbars", 130, 255, nothing)
cv2.createTrackbar("Y_V_max", "Trackbars", 210, 255, nothing)

# === TRACKBAR BLU ===
cv2.createTrackbar("B_H_min", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("B_H_max", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("B_S_min", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("B_S_max", "Trackbars", 162, 255, nothing)
cv2.createTrackbar("B_V_min", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("B_V_max", "Trackbars", 60, 255, nothing)

while True:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Leggi trackbar GIALLO
    yhmin = cv2.getTrackbarPos("Y_H_min", "Trackbars")
    yhmax = cv2.getTrackbarPos("Y_H_max", "Trackbars")
    ysmin = cv2.getTrackbarPos("Y_S_min", "Trackbars")
    ysmax = cv2.getTrackbarPos("Y_S_max", "Trackbars")
    yvmin = cv2.getTrackbarPos("Y_V_min", "Trackbars")
    yvmax = cv2.getTrackbarPos("Y_V_max", "Trackbars")

    # Leggi trackbar BLU
    bhmin = cv2.getTrackbarPos("B_H_min", "Trackbars")
    bhmax = cv2.getTrackbarPos("B_H_max", "Trackbars")
    bsmin = cv2.getTrackbarPos("B_S_min", "Trackbars")
    bsmax = cv2.getTrackbarPos("B_S_max", "Trackbars")
    bvmin = cv2.getTrackbarPos("B_V_min", "Trackbars")
    bvmax = cv2.getTrackbarPos("B_V_max", "Trackbars")

    # Maschere
    yellow_mask = cv2.inRange(hsv, (yhmin, ysmin, yvmin), (yhmax, ysmax, yvmax))
    blue_mask   = cv2.inRange(hsv, (bhmin, bsmin, bvmin), (bhmax, bsmax, bvmax))
    combined_mask = cv2.bitwise_or(yellow_mask, blue_mask)

    yellow_result = cv2.bitwise_and(image, image, mask=yellow_mask)
    blue_result   = cv2.bitwise_and(image, image, mask=blue_mask)
    combined_result = cv2.bitwise_and(image, image, mask=combined_mask)

    # Visualizzazione
    cv2.imshow("Yellow Mask", yellow_mask)
    cv2.imshow("Blue Mask", blue_mask)
    cv2.imshow("Combined", combined_result)

    key = cv2.waitKey(1)
    if key == ord("q"):
        break
    elif key == ord("s"):
        print("\n=== PARAMETRI CORRENTI ===")
        print(f"--yH_min={yhmin} --yH_max={yhmax} --yS_min={ysmin} --yS_max={ysmax} --yV_min={yvmin} --yV_max={yvmax}")
        print(f"--bH_min={bhmin} --bH_max={bhmax} --bS_min={bsmin} --bS_max={bsmax} --bV_min={bvmin} --bV_max={bvmax}")
        print("==========================\n")

cv2.destroyAllWindows()
