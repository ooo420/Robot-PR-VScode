"""
main.py

Programme principal du robot PR.

Gère la machine à états :
INIT
HOMING
WAIT_START
PICK
VACUUM
PLACE
RELEASE
RETURN
ALARM
"""

import time
import pigpio

import config
from motor_control import creer_axes, LimiteCourseError
from pneumatic import SystemePneumatique, DepressionError
from vision import VisionSystem
from mgi import appliquer_correction


# ============================================================
# CENTRE THEORIQUE CAMERA
# ============================================================

CENTRE_THEORIQUE_PX = (
    config.RESOLUTION_CAMERA[0] / 2,
    config.RESOLUTION_CAMERA[1] / 2
)


# ============================================================
# ETATS
# ============================================================

ETATS = [
    "INIT",
    "HOMING",
    "WAIT_START",
    "PICK",
    "VACUUM",
    "PLACE",
    "RELEASE",
    "RETURN",
    "ALARM"
]


# ============================================================
# ROBOT PR
# ============================================================

class RobotPR:

    def __init__(self):

        # Connexion au démon pigpio
        self.pi = pigpio.pi()

        if not self.pi.connected:

            raise RuntimeError(
                "Impossible de se connecter "
                "à pigpio."
            )

        # Création des axes
        self.axe_z, self.axe_theta = (
            creer_axes(self.pi)
        )

        # Système pneumatique
        self.pneumatique = (
            SystemePneumatique(self.pi)
        )

        # Système de vision
        self.vision = VisionSystem()

        # Bouton départ
        self.pi.set_mode(
            config.PIN_BOUTON_DEPART,
            pigpio.INPUT
        )

        self.pi.set_pull_up_down(
            config.PIN_BOUTON_DEPART,
            pigpio.PUD_UP
        )

        # LED alarme
        self.pi.set_mode(
            config.PIN_LED_ALARME,
            pigpio.OUTPUT
        )

        # Bouton acquittement
        self.pi.set_mode(
            config.PIN_BOUTON_ACQUITTEMENT,
            pigpio.INPUT
        )

        self.pi.set_pull_up_down(
            config.PIN_BOUTON_ACQUITTEMENT,
            pigpio.PUD_UP
        )

        # Etat initial
        self.etat = "INIT"

        # Temps de cycle
        self.debut_cycle = None


    # ========================================================
    # BOUCLE PRINCIPALE
    # ========================================================

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

            except (
                LimiteCourseError,
                DepressionError
            ) as erreur:

                print(
                    f"[ALARME] {erreur}"
                )

                self.etat = "ALARM"


    # ========================================================
    # INIT
    # ========================================================

    def _etat_init(self):

        print("[ETAT] INIT")

        # LED éteinte
        self.pi.write(
            config.PIN_LED_ALARME,
            0
        )

        # Désactivation moteurs
        self.axe_z.enable(False)
        self.axe_theta.enable(False)

        # Passage au homing
        self.etat = "HOMING"


    # ========================================================
    # HOMING
    # ========================================================

    def _etat_homing(self):

        print("[ETAT] HOMING")

        # Référencement axe Z
        self.axe_z.homing()

        # Référencement axe theta
        self.axe_theta.homing()

        print(
            "[HOMING] Référencement terminé."
        )

        self.etat = "WAIT_START"


    # ========================================================
    # ATTENTE DEPART
    # ========================================================

    def _etat_wait_start(self):

        print(
            "[ETAT] WAIT_START"
        )

        print(
            "En attente du bouton START..."
        )

        while (
            self.pi.read(
                config.PIN_BOUTON_DEPART
            ) != 0
        ):

            time.sleep(0.01)

        self.debut_cycle = time.time()

        print(
            "[SYSTEME] Cycle démarré."
        )

        self.etat = "PICK"


    # ========================================================
    # PICK
    # ========================================================

    def _etat_pick(self):

        print("[ETAT] PICK")

        position = config.POSITION_PICK

        # Rotation vers la position de prise
        self.axe_theta.aller_a(
            position["theta_deg"]
        )

        # Descente vers le LCD
        self.axe_z.aller_a(
            position["z_mm"]
        )

        self.etat = "VACUUM"


    # ========================================================
    # VACUUM
    # ========================================================

    def _etat_vacuum(self):

        print("[ETAT] VACUUM")

        # Activation + vérification
        self.pneumatique.prise_avec_verification()

        self.etat = "PLACE"


    # ========================================================
    # PLACE
    # ========================================================

    def _etat_place(self):

        print("[ETAT] PLACE")

        position = config.POSITION_PLACE

        # Position théorique de dépose
        self.axe_theta.aller_a(
            position["theta_deg"]
        )

        self.axe_z.aller_a(
            position["z_mm"]
        )

        # Capture image
        image = self.vision.capturer_image()

        # Calcul offset
        offset = (
            self.vision.calculer_offset_mm(
                image
            )
        )

        # ----------------------------------------------------
        # Cas où la vision détecte le logement
        # ----------------------------------------------------

        if offset is not None:

            dx_mm, dz_mm = offset

            if self.vision.offset_depasse_seuil(
                dx_mm,
                dz_mm
            ):

                print(
                    "[VISION] "
                    "Correction nécessaire."
                )

                # Correction par MGD + MGI
                theta_corrige, z_corrige = (
                    appliquer_correction(
                        self.axe_theta.position,
                        self.axe_z.position,
                        dx_mm,
                        dz_mm
                    )
                )

                # Déplacement corrigé
                self.axe_theta.aller_a(
                    theta_corrige
                )

                self.axe_z.aller_a(
                    z_corrige
                )

            else:

                print(
                    "[VISION] "
                    "Offset inférieur au seuil."
                )

        else:

            print(
                "[VISION] "
                "Pas de correction appliquée."
            )

        self.etat = "RELEASE"


    # ========================================================
    # RELEASE
    # ========================================================

    def _etat_release(self):

        print("[ETAT] RELEASE")

        # Arrêt du vide
        self.pneumatique.relacher()

        # Petit délai pour assurer le relâchement
        time.sleep(0.2)

        self.etat = "RETURN"


    # ========================================================
    # RETOUR HOME
    # ========================================================

    def _etat_return(self):

        print("[ETAT] RETURN")

        position = config.POSITION_HOME

        # Retour angle 0°
        self.axe_theta.aller_a(
            position["theta_deg"]
        )

        # Remontée
        self.axe_z.aller_a(
            position["z_mm"]
        )

        # Calcul durée cycle
        if self.debut_cycle is not None:

            duree = (
                time.time()
                - self.debut_cycle
            )

            print(
                f"[CYCLE] Durée = "
                f"{duree:.2f} s"
            )

            if duree <= config.CYCLE_MAX_S:

                print(
                    "[CYCLE] "
                    "Objectif respecté."
                )

            else:

                print(
                    "[CYCLE] "
                    "Objectif dépassé."
                )

        self.etat = "WAIT_START"


    # ========================================================
    # ALARME
    # ========================================================

    def _etat_alarm(self):

        print("[ETAT] ALARM")

        # Arrêt moteurs
        self.axe_z.enable(False)
        self.axe_theta.enable(False)

        # Arrêt pneumatique
        self.pneumatique.desactiver_vanne()

        # Allumage LED
        self.pi.write(
            config.PIN_LED_ALARME,
            1
        )

        print(
            "Appuyer sur le bouton "
            "d'acquittement..."
        )

        while (
            self.pi.read(
                config.PIN_BOUTON_ACQUITTEMENT
            ) != 0
        ):

            time.sleep(0.01)

        # Extinction LED
        self.pi.write(
            config.PIN_LED_ALARME,
            0
        )

        self.etat = "INIT"


# ============================================================
# PROGRAMME PRINCIPAL
# ============================================================

if __name__ == "__main__":

    robot = RobotPR()

    try:

        robot.run()

    except KeyboardInterrupt:

        print(
            "\nArrêt demandé par l'utilisateur."
        )

    finally:

        robot.pneumatique.desactiver_vanne()

        robot.axe_z.enable(False)
        robot.axe_theta.enable(False)

        robot.vision.fermer()

        robot.pi.stop()

        print(
            "Système arrêté proprement."
        )
