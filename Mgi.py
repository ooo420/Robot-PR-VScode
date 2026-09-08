"""
mgi.py
Implémente le Modèle Géométrique Inverse (MGI) et Direct (MGD) du robot
PR (Pivot + translation), conformément à la dérivation DH du rapport
(§2.5.2 - 2.5.3) :

    x = L1 + L2·cos(θ)
    z = q1 + L2·sin(θ)

où L1 (config.LONGUEUR_L1_MM) est l'offset fixe de la base (bras 1,
prismatique) et L2 (config.LONGUEUR_BRAS_MM) la longueur du bras 2
(rotation theta).
"""

import math
import config


class PositionInaccessibleError(Exception):
    """Levée quand la position cartésienne demandée est hors d'atteinte
    du bras (condition de portée non respectée) ou hors de la plage de
    course de l'axe Z."""


def mgd(q1_mm: float, theta_deg: float):
    """
    Modèle Géométrique Direct : retourne la position cartésienne (x, z)
    de l'effecteur à partir de la consigne articulaire (q1, theta).
    """
    L1 = config.LONGUEUR_L1_MM
    L2 = config.LONGUEUR_BRAS_MM
    theta_rad = math.radians(theta_deg)

    x_mm = L1 + L2 * math.cos(theta_rad)
    z_mm = q1_mm + L2 * math.sin(theta_rad)
    return x_mm, z_mm


def mgi(x_mm: float, z_mm: float):
    """
    Modèle Géométrique Inverse : calcule la consigne articulaire
    (q1, theta) à partir des coordonnées cartésiennes (x, z) désirées.

    Condition de portée : (x-L1)² + (z-q1)² = L2²
        => q1 = z ± sqrt(L2² - (x-L1)²)
        => theta = atan2(z - q1, x - L1)

    Le signe ± correspond aux deux configurations possibles du bras ;
    on retient celle qui respecte la plage de course de l'axe Z
    (config.Q1_MIN_MM <= q1 <= config.Q1_MAX_MM).
    """
    L1 = config.LONGUEUR_L1_MM
    L2 = config.LONGUEUR_BRAS_MM

    dx = x_mm - L1
    discriminant = L2 ** 2 - dx ** 2
    if discriminant < 0:
        raise PositionInaccessibleError(
            f"x={x_mm} mm hors de portée du bras (L1={L1} mm, L2={L2} mm)."
        )
    racine = math.sqrt(discriminant)

    for q1_candidat in (z_mm - racine, z_mm + racine):
        if config.Q1_MIN_MM <= q1_candidat <= config.Q1_MAX_MM:
            theta_rad = math.atan2(z_mm - q1_candidat, dx)
            return q1_candidat, math.degrees(theta_rad)

    raise PositionInaccessibleError(
        f"Aucune solution MGI pour (x={x_mm}, z={z_mm}) mm ne respecte "
        f"la course de l'axe Z [{config.Q1_MIN_MM}, {config.Q1_MAX_MM}] mm."
    )


def appliquer_correction(theta_actuel_deg: float, z_actuel_mm: float,
                          dx_mm: float, dz_mm: float):
    """
    Recalcule la consigne (q1, theta) corrigée à partir de l'offset
    visuel (dx, dz) mesuré par vision.py, lorsque celui-ci dépasse le
    seuil de tolérance (config.SEUIL_OFFSET_MM). L'offset est ajouté à
    la position cartésienne courante avant de repasser par le MGI.
    """
    x_actuel_mm, _ = mgd(z_actuel_mm, theta_actuel_deg)
    x_corrige = x_actuel_mm + dx_mm
    z_corrige = z_actuel_mm + dz_mm
    return mgi(x_corrige, z_corrige)
