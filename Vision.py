"""
vision.py
Utilise la Pi Camera V2 pour capturer une image du support, détecter
les contours du logement destiné à recevoir le LCD, et calculer
l'offset (dx, dz) en millimètres entre le centre réel du logement et
la position théorique attendue.
"""
 
import cv2
import numpy as np
from picamera2 import Picamera2
 
import config
 
 
class VisionSystem:
    def __init__(self):
        self.camera = Picamera2()
        cfg = self.camera.create_still_configuration(
            main={"size": config.RESOLUTION_CAMERA}
        )
        self.camera.configure(cfg)
        self.camera.start()
 
    # ------------------------------------------------------------------ #
    def capturer_image(self):
        """Capture une image (tableau numpy RGB) juste avant l'étape PLACE."""
        return self.camera.capture_array()
 
    # ------------------------------------------------------------------ #
    def detecter_logement(self, image):
        """
        1. Conversion en niveaux de gris et seuillage adaptatif.
        2. Détection des contours du logement destiné à recevoir le LCD.
        3. Calcul du centre du rectangle détecté.
        Retourne (cx, cy) en pixels, ou None si aucun contour valable
        n'est trouvé.
        """
        gris = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        gris = cv2.GaussianBlur(gris, (5, 5), 0)
 
        seuil = cv2.adaptiveThreshold(
            gris, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            blockSize=21, C=5,
        )
 
        contours, _ = cv2.findContours(seuil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
 
        # on retient le plus grand contour rectangulaire plausible
        contour_principal = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(contour_principal)
 
        cx = x + w / 2.0
        cy = y + h / 2.0
        return cx, cy
 
    # ------------------------------------------------------------------ #
    def calculer_offset_mm(self, image, centre_theorique_px):
        """
        4. Estimation de l'offset (dx, dz) en millimètres grâce au
        facteur d'échelle pré-étalonné (FACTEUR_ECHELLE_PX_PAR_MM).
        Retourne (dx_mm, dz_mm) ou None si la détection a échoué.
        """
        centre_detecte = self.detecter_logement(image)
        if centre_detecte is None:
            return None
 
        cx, cy = centre_detecte
        cx_theo, cy_theo = centre_theorique_px
 
        dx_px = cx - cx_theo
        dz_px = cy - cy_theo
 
        dx_mm = dx_px / config.FACTEUR_ECHELLE_PX_PAR_MM
        dz_mm = dz_px / config.FACTEUR_ECHELLE_PX_PAR_MM
        return dx_mm, dz_mm
 
    # ------------------------------------------------------------------ #
    def offset_depasse_seuil(self, dx_mm, dz_mm) -> bool:
        """Vrai si l'offset dépasse le seuil de tolérance (0,05 mm)."""
        return (dx_mm ** 2 + dz_mm ** 2) ** 0.5 > config.SEUIL_OFFSET_MM
 
    # ------------------------------------------------------------------ #
    def fermer(self):
        self.camera.stop()