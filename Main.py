"""
config.py
Centralise tous les paramètres matériels et logiciels du robot PR
(broches GPIO, résolutions angulaire et linéaire, positions de
prise/dépose, seuils de sécurité). Aucune "valeur magique" ne doit
être codée en dur ailleurs dans le projet : tout passe par ce module.

Brochage GPIO conforme au Tableau 27 / Tableau 30 / §3.4 du rapport.
"""

# ---------------------------------------------------------------------------
# Broches GPIO (numérotation BCM) — Tableau 27
# ---------------------------------------------------------------------------

# Axe Z (translation) — moteur NEMA 23 + vis à billes G1610
PIN_Z_STEP = 17
PIN_Z_DIR = 27
PIN_Z_ENABLE = 22
PIN_Z_LIMIT_BAS = 5        # SENS_HOME_Z : capteur inductif bas (référence homing, q1=170 mm)
PIN_Z_LIMIT_HAUT = 6       # SENS_LIMIT_Z : capteur inductif haut (q1=370 mm)

# Axe theta (rotation) — moteur NEMA 17 + réducteur harmonique PG14-50 (50:1)
PIN_T_STEP = 23
PIN_T_DIR = 24
PIN_T_ENABLE = 25
PIN_T_ZERO = 13             # SENS_HOME_θ : capteur θ = 0° (référence homing)
PIN_T_LIMIT_ROTATION = 19   # SENS_LIMIT_θ : capteur limite de rotation

# Système pneumatique (préhension par ventouse silicone)
PIN_VANNE_SSR = 26          # RELAY_SOL : commande CH1 du SSR (électrovanne), actif HIGH
PIN_CAPTEUR_PRESSION = 20   # SENS_VACUUM : capteur de pression NPN (0 = vide détecté)

# Circuit de sécurité (§3.4)
PIN_RETOUR_SECURITE = 21    # RETOUR_SECURITE : retour d'état du relais XPS-AC (via PC817)

# Signalisation lumineuse (Tableau 37)
PIN_LED_VERTE = 4           # LED_GREEN : voyant "prêt"
PIN_LED_ALARME = 18         # LED_RED : voyant "défaut"
PIN_LED_PWM_ANNEAU = 12     # PWM_LED : anneau LED (PWM)

# Boutons opérateur — NON documentés dans le Tableau 27 du rapport.
# ⚠️ À vérifier / adapter selon le câblage réel avant utilisation.
PIN_BOUTON_DEPART = 16
PIN_BOUTON_ACQUITTEMENT = 9

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
# Géométrie du robot PR (pour le MGI/MGD, §2.5.2-2.5.3)
# ---------------------------------------------------------------------------

LONGUEUR_L1_MM = 200.0                # L1 : offset fixe de la base (bras 1, prismatique)
LONGUEUR_BRAS_MM = 250.0              # L2 : longueur du bras 2 (rotation theta)

# Plage de course de l'axe Z dans le repère absolu du rapport
# (le capteur bas = q1=170 mm, le capteur haut = q1=370 mm)
Q1_MIN_MM = 170.0
Q1_MAX_MM = 370.0
OFFSET_HOMING_Z_MM = Q1_MIN_MM        # position réinjectée après homing de l'axe Z
OFFSET_HOMING_T_DEG = 0.0             # position réinjectée après homing de l'axe theta

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
