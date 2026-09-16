"""
vision.py

Système de vision basé sur Raspberry Pi Camera + OpenCV.

Fonctions :
- acquisition image
- détection du logement
- calcul du centre
- calcul de l'offset
- décision de correction
"""

import cv2
import numpy as np
from picamera2 import Picamera2
import config


# ============================================================
# SYSTEME DE VISION
# ============================================================

class VisionSystem:

    def __init__(self):

        # Création de la caméra
        self.camera = Picamera2()

        # Configuration image
        configuration = (
            self.camera.create_preview_configuration(
                main={
                    "size": config.RESOLUTION_CAMERA,
                    "format": "RGB888"
                }
            )
        )

        # Application de la configuration
        self.camera.configure(
            configuration
        )

        # Démarrage caméra
        self.camera.start()


    # ========================================================
    # CAPTURE
    # ========================================================

    def capturer_image(self):

        """
        Capture une image RGB.
        """

        image = self.camera.capture_array()

        return image


    # ========================================================
    # DETECTION DU LOGEMENT
    # ========================================================

    def detecter_logement(self, image):

        """
        Détecte le contour principal correspondant
        au logement du support.

        Retour :
            (cx, cy) si une détection est obtenue
            None sinon
        """

        # RGB -> niveaux de gris
        gris = cv2.cvtColor(
            image,
            cv2.COLOR_RGB2GRAY
        )

        # Réduction du bruit
        flou = cv2.GaussianBlur(
            gris,
            (5, 5),
            0
        )

        # Seuillage adaptatif
        seuil = cv2.adaptiveThreshold(
            flou,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            21,
            5
        )

        # Recherche des contours externes
        contours, _ = cv2.findContours(
            seuil,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return None

        # Tri par surface
        contours = sorted(
            contours,
            key=cv2.contourArea,
            reverse=True
        )

        # Recherche d'un contour suffisamment grand
        for contour in contours:

            aire = cv2.contourArea(
                contour
            )

            if aire < 100:
                continue

            x, y, w, h = cv2.boundingRect(
                contour
            )

            if w <= 0 or h <= 0:
                continue

            # Centre du rectangle
            cx = x + w / 2.0
            cy = y + h / 2.0

            return cx, cy

        return None


    # ========================================================
    # CALCUL OFFSET
    # ========================================================

    def calculer_offset_mm(self, image):

        """
        Calcule l'écart entre le centre détecté
        et le centre théorique de l'image.
        """

        centre_detecte = self.detecter_logement(
            image
        )

        if centre_detecte is None:

            print(
                "[VISION] "
                "Aucun logement détecté."
            )

            return None

        cx, cy = centre_detecte

        largeur, hauteur = (
            config.RESOLUTION_CAMERA
        )

        # Centre théorique
        cx_theorique = largeur / 2.0
        cy_theorique = hauteur / 2.0

        # Offset en pixels
        dx_px = (
            cx - cx_theorique
        )

        dz_px = (
            cy - cy_theorique
        )

        # Conversion pixels -> millimètres
        dx_mm = (
            dx_px
            / config.FACTEUR_ECHELLE_PX_PAR_MM
        )

        dz_mm = (
            dz_px
            / config.FACTEUR_ECHELLE_PX_PAR_MM
        )

        print(
            f"[VISION] Offset : "
            f"dx={dx_mm:.3f} mm, "
            f"dz={dz_mm:.3f} mm"
        )

        return dx_mm, dz_mm


    # ========================================================
    # DECISION
    # ========================================================

    def offset_depasse_seuil(
        self,
        dx_mm,
        dz_mm
    ):

        """
        Vérifie si l'erreur dépasse
        le seuil de correction.
        """

        erreur = (
            dx_mm**2
            + dz_mm**2
        ) ** 0.5

        return (
            erreur
            > config.SEUIL_OFFSET_MM
        )


    # ========================================================
    # FERMETURE
    # ========================================================

    def fermer(self):

        """
        Arrête la caméra.
        """

        self.camera.stop()

        print(
            "[VISION] Caméra arrêtée."
        )
