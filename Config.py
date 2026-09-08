"""
config.py
Centralise tous les paramètres matériels et logiciels du robot PR
(broches GPIO, résolutions angulaire et linéaire, positions de
prise/dépose, seuils de sécurité). Aucune "valeur magique" ne doit
être codée en dur ailleurs dans le projet : tout passe par ce module.
"""
 
# ---------------------------------------------------------------------------
# Broches GPIO (numérotation BCM)
# ---------------------------------------------------------------------------
 
# Axe Z (translation) — moteur NEMA 23 + vis à billes G1610
PIN_Z_STEP = 17
PIN_Z_DIR = 27
PIN_Z_ENABLE = 22
PIN_Z_LIMIT_HAUT = 5      # capteur inductif fin de course haute
PIN_Z_LIMIT_BAS = 6       # capteur inductif fin de course basse (référence homing)
 
# Axe theta (rotation) — moteur NEMA 17 + réducteur harmonique PG14-50 (50:1)
PIN_T_STEP = 23
PIN_T_DIR = 24
PIN_T_ENABLE = 25
PIN_T_LIMIT_ROTATION = 16  # capteur inductif limite de rotation
PIN_T_ZERO = 12            # capteur inductif référence theta = 0°
 
# Système pneumatique (préhension par ventouse silicone)
PIN_VANNE_SSR = 20         # commande relais SSR de l'électrovanne
PIN_CAPTEUR_PRESSION = 21  # capteur de pression NPN (0 = vide détecté)
 
# Divers
PIN_BOUTON_DEPART = 26
PIN_LED_ALARME = 19
PIN_BOUTON_ACQUITTEMENT = 13
 
# ---------------------------------------------------------------------------
# Résolutions
# ---------------------------------------------------------------------------
 
# Axe Z : vis à billes G1610 -> pas de 10 mm/tour, driver TMC2209 en micro-pas 1/16
PAS_PAR_TOUR_MOTEUR_Z = 200          # moteur NEMA 23, 1.8°/pas
MICROPAS_Z = 16
PAS_VIS_MM = 10.0                    # avancement (mm) par tour de vis
RESOLUTION_Z_MM = PAS_VIS_MM / (PAS_PAR_TOUR_MOTEUR_Z * MICROPAS_Z)   # mm par micro-pas
 
# Axe theta : moteur NEMA 17 (1.8°/pas) + réducteur 50:1, micro-pas 1/16
PAS_PAR_TOUR_MOTEUR_T = 200
MICROPAS_T = 16
RATIO_REDUCTEUR = 50.0
RESOLUTION_T_DEG = 360.0 / (PAS_PAR_TOUR_MOTEUR_T * MICROPAS_T * RATIO_REDUCTEUR)  # degrés par micro-pas
 
# ---------------------------------------------------------------------------
# Positions du cycle pick-and-place (cf. Tableau 39 du rapport)
# ---------------------------------------------------------------------------
 
POSITION_PICK = {"theta_deg": 30.0, "z_mm": 200.0}
POSITION_PLACE = {"theta_deg": 45.0, "z_mm": 190.0}
POSITION_HOME = {"theta_deg": 0.0, "z_mm": 370.0}
 
# ---------------------------------------------------------------------------
# Vitesses / rampe trapézoïdale
# ---------------------------------------------------------------------------
 
V_MAX_Z_MM_S = 80.0
A_MAX_Z_MM_S2 = 200.0
V_MAX_T_DEG_S = 90.0
A_MAX_T_DEG_S2 = 250.0
 
# ---------------------------------------------------------------------------
# Sécurité / temporisations
# ---------------------------------------------------------------------------
 
VACUUM_ESSAIS_MAX = 3
VACUUM_TIMEOUT_S = 1.0
CYCLE_MAX_S = 7.0                    # cadence cible (<= 15 s/cycle au cahier des charges)
 
# ---------------------------------------------------------------------------
# Vision embarquée
# ---------------------------------------------------------------------------
 
SEUIL_OFFSET_MM = 0.05                # au-delà : recalcul MGI et repositionnement
FACTEUR_ECHELLE_PX_PAR_MM = 11.8      # facteur d'échelle pré-étalonné (pixels/mm), Pi Camera V2
RESOLUTION_CAMERA = (1280, 720)
 
# ---------------------------------------------------------------------------
# Géométrie du robot PR (pour le MGI)
# ---------------------------------------------------------------------------
 
LONGUEUR_BRAS_MM = 250.0              # longueur du bras portant l'effecteur (rotation theta)