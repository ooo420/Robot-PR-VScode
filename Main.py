"""
main.py
Contient la machine d'états finie (FSM) qui orchestre le cycle
pick-and-place complet du robot PR (Tableau 39 du rapport) :
 
INIT -> HOMING -> WAIT_START -> PICK -> VACUUM -> PLACE -> RELEASE
-> RETURN -> WAIT_START (boucle), avec un état ALARM accessible
depuis n'importe quel état en cas d'anomalie.
"""
 
import time
import pigpio
 
import config
from motor_control import creer_axes, LimiteCourseError
from pneumatic import SystemePneumatique, DepressionError
from vision import VisionSystem
from mgi import mgi, appliquer_correction
 
# Centre théorique du logement dans l'image, en pixels (calibré une fois
# la caméra positionnée au-dessus du poste de dépose).
CENTRE_THEORIQUE_PX = (config.RESOLUTION_CAMERA[0] / 2, config.RESOLUTION_CAMERA[1] / 2)
 
ETATS = [
    "INIT", "HOMING", "WAIT_START", "PICK", "VACUUM",
    "PLACE", "RELEASE", "RETURN", "ALARM",
]
 
 
class RobotPR:
    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Impossible de se connecter au démon pigpiod.")
 
        self.axe_z, self.axe_theta = creer_axes(self.pi)
        self.pneumatique = SystemePneumatique(self.pi)
        self.vision = VisionSystem()
 
        self.pi.set_mode(config.PIN_BOUTON_DEPART, pigpio.INPUT)
        self.pi.set_pull_up_down(config.PIN_BOUTON_DEPART, pigpio.PUD_UP)
        self.pi.set_mode(config.PIN_LED_ALARME, pigpio.OUTPUT)
        self.pi.set_mode(config.PIN_BOUTON_ACQUITTEMENT, pigpio.INPUT)
        self.pi.set_pull_up_down(config.PIN_BOUTON_ACQUITTEMENT, pigpio.PUD_UP)
 
        self.etat = "INIT"
 
    # ------------------------------------------------------------------ #
    # Boucle principale de la FSM
    # ------------------------------------------------------------------ #
    def run(self):
        while True:
            try:
                if self.etat == "INIT":
                    self._etat_init()
                elif self.etat == "HOMING":
                    self._etat_homing()
                elif self.etat == "WAIT_START":
                    self._etat_wait_start()
                elif self.etat == "PICK":
                    self._etat_pick()
                elif self.etat == "VACUUM":
                    self._etat_vacuum()
                elif self.etat == "PLACE":
                    self._etat_place()
                elif self.etat == "RELEASE":
                    self._etat_release()
                elif self.etat == "RETURN":
                    self._etat_return()
                elif self.etat == "ALARM":
                    self._etat_alarm()
            except (LimiteCourseError, DepressionError) as erreur:
                print(f"[ALARME] {erreur}")
                self.etat = "ALARM"
 
    # ------------------------------------------------------------------ #
    # INIT : initialisation des GPIO et des drivers -> HOMING (automatique)
    # ------------------------------------------------------------------ #
    def _etat_init(self):
        print("[INIT] Initialisation des GPIO et des drivers.")
        self.pi.write(config.PIN_LED_ALARME, 0)
        self.axe_z.enable(False)
        self.axe_theta.enable(False)
        self.etat = "HOMING"
 
    # ------------------------------------------------------------------ #
    # HOMING : référencement Z puis theta -> WAIT_START (capteurs détectés)
    # ------------------------------------------------------------------ #
    def _etat_homing(self):
        print("[HOMING] Référencement axe Z (capteur bas)...")
        self.axe_z.homing(sens=-1)
        print("[HOMING] Référencement axe theta (capteur 0°)...")
        self.axe_theta.homing(sens=-1)
        self.etat = "WAIT_START"
 
    # ------------------------------------------------------------------ #
    # WAIT_START : attente du signal "Départ cycle" -> PICK (bouton départ)
    # ------------------------------------------------------------------ #
    def _etat_wait_start(self):
        print("[WAIT_START] En attente du bouton départ...")
        while self.pi.read(config.PIN_BOUTON_DEPART) == 1:
            time.sleep(0.05)
        self._debut_cycle = time.time()
        self.etat = "PICK"
 
    # ------------------------------------------------------------------ #
    # PICK : déplacement vers theta=30°, Z=200 mm -> VACUUM (position atteinte)
    # ------------------------------------------------------------------ #
    def _etat_pick(self):
        pos = config.POSITION_PICK
        print(f"[PICK] Déplacement vers theta={pos['theta_deg']}°, Z={pos['z_mm']} mm.")
        self.axe_theta.aller_a(pos["theta_deg"])
        self.axe_z.aller_a(pos["z_mm"])
        self.etat = "VACUUM"
 
    # ------------------------------------------------------------------ #
    # VACUUM : activation électrovanne, vérification dépression
    #          (3 essais, timeout 1 s) -> PLACE (vide confirmé)
    # ------------------------------------------------------------------ #
    def _etat_vacuum(self):
        print("[VACUUM] Activation électrovanne et vérification dépression...")
        self.pneumatique.prise_avec_verification()
        print("[VACUUM] Vide confirmé.")
        self.etat = "PLACE"
 
    # ------------------------------------------------------------------ #
    # PLACE : déplacement vers theta=45°, Z=190 mm + correction visuelle
    #         -> RELEASE (position atteinte)
    # ------------------------------------------------------------------ #
    def _etat_place(self):
        pos = config.POSITION_PLACE
        print(f"[PLACE] Déplacement vers theta={pos['theta_deg']}°, Z={pos['z_mm']} mm.")
        self.axe_theta.aller_a(pos["theta_deg"])
        self.axe_z.aller_a(pos["z_mm"])
 
        print("[PLACE] Capture image et correction par vision embarquée...")
        image = self.vision.capturer_image()
        offset = self.vision.calculer_offset_mm(image, CENTRE_THEORIQUE_PX)
 
        if offset is not None:
            dx_mm, dz_mm = offset
            if self.vision.offset_depasse_seuil(dx_mm, dz_mm):
                print(f"[PLACE] Offset ({dx_mm:.3f}, {dz_mm:.3f}) mm > seuil "
                      f"({config.SEUIL_OFFSET_MM} mm) : recalcul MGI.")
                q1_corrige, theta_corrige = appliquer_correction(
                    self.axe_theta.position, self.axe_z.position, dx_mm, dz_mm
                )
                self.axe_theta.aller_a(theta_corrige)
                self.axe_z.aller_a(q1_corrige)
            else:
                print("[PLACE] Offset dans la tolérance, pas de correction nécessaire.")
        else:
            print("[PLACE] Logement non détecté : dépose à la position théorique.")
 
        self.etat = "RELEASE"
 
    # ------------------------------------------------------------------ #
    # RELEASE : désactivation électrovanne, attente libération
    #           -> RETURN (libération confirmée)
    # ------------------------------------------------------------------ #
    def _etat_release(self):
        print("[RELEASE] Désactivation électrovanne.")
        self.pneumatique.relacher()
        time.sleep(0.2)   # temporisation de libération
        self.etat = "RETURN"
 
    # ------------------------------------------------------------------ #
    # RETURN : retour position home (Z=370 mm, theta=0°)
    #          -> WAIT_START (position atteinte)
    # ------------------------------------------------------------------ #
    def _etat_return(self):
        pos = config.POSITION_HOME
        print(f"[RETURN] Retour position home (Z={pos['z_mm']} mm, theta={pos['theta_deg']}°).")
        self.axe_theta.aller_a(pos["theta_deg"])
        self.axe_z.aller_a(pos["z_mm"])
 
        duree_cycle = time.time() - self._debut_cycle
        print(f"[RETURN] Cycle terminé en {duree_cycle:.2f} s "
              f"(cible <= {config.CYCLE_MAX_S} s).")
        self.etat = "WAIT_START"
 
    # ------------------------------------------------------------------ #
    # ALARM : arrêt d'urgence, coupure actionneurs, LED rouge
    #         -> INIT (acquittement opérateur)
    # ------------------------------------------------------------------ #
    def _etat_alarm(self):
        print("[ALARM] Arrêt d'urgence : coupure des actionneurs.")
        self.axe_z.enable(False)
        self.axe_theta.enable(False)
        self.pneumatique.desactiver_vanne()
        self.pi.write(config.PIN_LED_ALARME, 1)
 
        print("[ALARM] En attente de l'acquittement opérateur...")
        while self.pi.read(config.PIN_BOUTON_ACQUITTEMENT) == 1:
            time.sleep(0.1)
 
        self.pi.write(config.PIN_LED_ALARME, 0)
        self.etat = "INIT"
 
 
if __name__ == "__main__":
    robot = RobotPR()
    robot.run()
 
