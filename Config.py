"""
config.py
Centralise tous les paramètres matériels et logiciels du robot PR :
- broches GPIO
- résolutions
- limites mécaniques
- positions de travail
- vitesses et accélérations
- paramètres pneumatiques
- paramètres de vision

Toutes les valeurs importantes sont regroupées ici afin d'éviter
les "valeurs magiques" dans les autres fichiers.
"""

# ============================================================
# GPIO BCM
# ============================================================

# Axe prismatique Z
PIN_Z_STEP = 17
PIN_Z_DIR = 27
PIN_Z_ENABLE = 22
PIN_Z_LIMIT_HAUT = 5
PIN_Z_LIMIT_BAS = 6

# Axe rotatif theta
PIN_T_STEP = 23
PIN_T_DIR = 24
PIN_T_ENABLE = 25
PIN_T_LIMIT_ROTATION = 16
PIN_T_ZERO = 12

# Pneumatique
PIN_VANNE_SSR = 20
PIN_CAPTEUR_PRESSION = 21

# Commandes utilisateur
PIN_BOUTON_DEPART = 26
PIN_LED_ALARME = 19
PIN_BOUTON_ACQUITTEMENT = 13


# ============================================================
# PARAMETRES MECANIQUES
# ============================================================

# Longueurs du robot
LONGUEUR_L1_MM = 200.0
LONGUEUR_BRAS_MM = 250.0

# Course de l'axe prismatique
Q1_MIN_MM = 170.0
Q1_MAX_MM = 370.0

# Offset utilisé après homing
OFFSET_HOMING_Z_MM = 170.0
OFFSET_HOMING_T_DEG = 0.0


# ============================================================
# RESOLUTION AXE Z
# ============================================================

PAS_PAR_TOUR_MOTEUR_Z = 200
MICROPAS_Z = 16
PAS_VIS_MM = 10.0

# Déplacement correspondant à un micro-pas
RESOLUTION_Z_MM = PAS_VIS_MM / (
    PAS_PAR_TOUR_MOTEUR_Z * MICROPAS_Z
)


# ============================================================
# RESOLUTION AXE ROTATIF
# ============================================================

PAS_PAR_TOUR_MOTEUR_T = 200
MICROPAS_T = 16
RATIO_REDUCTEUR = 50.0

# Angle correspondant à un micro-pas
RESOLUTION_T_DEG = 360.0 / (
    PAS_PAR_TOUR_MOTEUR_T
    * MICROPAS_T
    * RATIO_REDUCTEUR
)


# ============================================================
# POSITIONS DE TRAVAIL
# ============================================================

# Position de prise du LCD
POSITION_PICK = {
    "theta_deg": 0.0,
    "z_mm": 178.0
}

# Position de dépose sur le support
POSITION_PLACE = {
    "theta_deg": 45.0,
    "z_mm": 178.0
}

# Position de référence
POSITION_HOME = {
    "theta_deg": 0.0,
    "z_mm": 370.0
}


# ============================================================
# LIMITES DE MOUVEMENT
# ============================================================

V_MAX_Z_MM_S = 80.0
A_MAX_Z_MM_S2 = 200.0

V_MAX_T_DEG_S = 90.0
A_MAX_T_DEG_S2 = 250.0


# ============================================================
# SYSTEME DE VIDE
# ============================================================

VACUUM_ESSAIS_MAX = 3
VACUUM_TIMEOUT_S = 1.0


# ============================================================
# CYCLE
# ============================================================

CYCLE_MAX_S = 7.0


# ============================================================
# VISION
# ============================================================

SEUIL_OFFSET_MM = 0.05

# 11.8 pixels correspondent à 1 mm
FACTEUR_ECHELLE_PX_PAR_MM = 11.8

# Résolution de traitement
RESOLUTION_CAMERA = (1280, 720)
