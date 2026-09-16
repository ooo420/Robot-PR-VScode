"""
mgi.py

Contient les fonctions de cinématique du robot PR :
- MGD : coordonnées articulaires -> coordonnées cartésiennes
- MGI : coordonnées cartésiennes -> coordonnées articulaires
- correction de position à partir de la vision
"""

import math
import config


# ============================================================
# EXCEPTION
# ============================================================

class PositionInaccessibleError(Exception):
    """Levée lorsqu'une position demandée est inaccessible."""
    pass


# ============================================================
# MGD
# ============================================================

def mgd(q1_mm, theta_deg):
    """
    Modèle géométrique direct.

    Entrées :
        q1_mm    : position de l'axe prismatique [mm]
        theta_deg: angle de rotation [deg]

    Sorties :
        x_mm     : position cartésienne X [mm]
        z_mm     : position cartésienne Z [mm]
    """

    # Récupération des dimensions
    L1 = config.LONGUEUR_L1_MM
    L2 = config.LONGUEUR_BRAS_MM

    # Conversion degrés -> radians
    theta_rad = math.radians(theta_deg)

    # Calcul de la position cartésienne
    x_mm = L1 + L2 * math.cos(theta_rad)
    z_mm = q1_mm + L2 * math.sin(theta_rad)

    return x_mm, z_mm


# ============================================================
# MGI
# ============================================================

def mgi(x_mm, z_mm):
    """
    Modèle géométrique inverse.

    Entrées :
        x_mm : position cartésienne X [mm]
        z_mm : position cartésienne Z [mm]

    Sorties :
        q1_mm     : position de l'axe Z [mm]
        theta_deg : angle de rotation [deg]
    """

    L1 = config.LONGUEUR_L1_MM
    L2 = config.LONGUEUR_BRAS_MM

    # Distance horizontale entre L1 et la position demandée
    dx = x_mm - L1

    # Équation géométrique :
    # dx² + (z-q1)² = L2²
    discriminant = L2**2 - dx**2

    # Vérification de l'accessibilité
    if discriminant < 0:
        raise PositionInaccessibleError(
            f"Position inaccessible : X={x_mm:.2f} mm, "
            f"Z={z_mm:.2f} mm"
        )

    # Deux configurations géométriques possibles
    racine = math.sqrt(discriminant)

    q1_candidats = (
        z_mm - racine,
        z_mm + racine
    )

    # Recherche d'une solution respectant les limites
    for q1_candidat in q1_candidats:

        if (
            config.Q1_MIN_MM
            <= q1_candidat
            <= config.Q1_MAX_MM
        ):

            theta_rad = math.atan2(
                z_mm - q1_candidat,
                dx
            )

            theta_deg = math.degrees(theta_rad)

            # Vérification de l'angle
            if -90.0 <= theta_deg <= 90.0:
                return q1_candidat, theta_deg

    # Aucune solution valide
    raise PositionInaccessibleError(
        f"Aucune configuration valide pour "
        f"X={x_mm:.2f} mm, Z={z_mm:.2f} mm"
    )


# ============================================================
# CORRECTION PAR VISION
# ============================================================

def appliquer_correction(
    theta_actuel_deg,
    z_actuel_mm,
    dx_mm,
    dz_mm
):
    """
    Applique la correction provenant du système de vision.

    Étapes :
        1. Calcul de la position cartésienne actuelle avec MGD
        2. Ajout des offsets détectés par la caméra
        3. Calcul des nouvelles coordonnées articulaires avec MGI
    """

    # Position cartésienne actuelle
    x_actuel, z_cartesien = mgd(
        z_actuel_mm,
        theta_actuel_deg
    )

    # Correction demandée par la vision
    x_corrige = x_actuel + dx_mm
    z_corrige = z_cartesien + dz_mm

    # Retour dans l'espace articulaire
    q1_corrige, theta_corrige = mgi(
        x_corrige,
        z_corrige
    )

    return theta_corrige, q1_corrige
