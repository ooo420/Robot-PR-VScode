"""
mgi.py
Implémente le Modèle Géométrique Inverse (MGI) du robot PR (Pivot +
translation) : calcule la consigne articulaire (q1, theta) à partir
des coordonnées cartésiennes (x, z) souhaitées pour l'effecteur.
 
Convention : l'axe theta fait pivoter le bras (longueur fixe L =
config.LONGUEUR_BRAS_MM) dans le plan horizontal ; l'axe q1 (= Z)
translate verticalement la nacelle qui porte ce bras.
"""
 
import math
 
import config
 
 
class PositionInaccessibleError(Exception):
    """Levée quand la coordonnée x demandée dépasse la longueur du bras."""
 
 
def mgi(x_mm: float, z_mm: float):
    """
    Calcule la consigne (q1, theta) à partir des coordonnées
    cartésiennes (x, z) de l'effecteur.
 
    - q1 (mm)      : consigne de l'axe de translation Z (identique à z_mm
                      dans cette architecture, le bras ne portant pas de
                      composante verticale propre).
    - theta (deg)  : angle du bras nécessaire pour atteindre x_mm,
                      calculé par trigonométrie inverse sur la longueur
                      de bras fixe L.
    """
    L = config.LONGUEUR_BRAS_MM
 
    if abs(x_mm) > L:
        raise PositionInaccessibleError(
            f"x={x_mm} mm hors de portée du bras (longueur = {L} mm)."
        )
 
    theta_rad = math.asin(x_mm / L)
    theta_deg = math.degrees(theta_rad)
    q1_mm = z_mm
 
    return q1_mm, theta_deg
 
 
def mgd(q1_mm: float, theta_deg: float):
    """
    Modèle Géométrique Direct (utilisé pour vérifier le MGI en test
    unitaire) : retourne (x, z) à partir de la consigne articulaire.
    """
    L = config.LONGUEUR_BRAS_MM
    theta_rad = math.radians(theta_deg)
    x_mm = L * math.sin(theta_rad)
    z_mm = q1_mm
    return x_mm, z_mm
 
 
def appliquer_correction(theta_actuel_deg: float, z_actuel_mm: float,
                          dx_mm: float, dz_mm: float):
    """
    Recalcule la consigne (q1, theta) corrigée à partir de l'offset
    visuel (dx, dz) mesuré par vision.py, lorsque celui-ci dépasse le
    seuil de tolérance (0,05 mm). L'offset est ajouté à la position
    cartésienne courante avant de repasser par le MGI.
    """
    x_actuel_mm, _ = mgd(z_actuel_mm, theta_actuel_deg)
    x_corrige = x_actuel_mm + dx_mm
    z_corrige = z_actuel_mm + dz_mm
    return mgi(x_corrige, z_corrige)