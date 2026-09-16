"""
motor_control.py

Gestion des moteurs pas-à-pas :
- activation/désactivation
- déplacement
- génération des impulsions STEP
- gestion de la direction
- homing
- limitation de course
- profil de vitesse
"""

import time
import pigpio
import config


# ============================================================
# EXCEPTION
# ============================================================

class LimiteCourseError(Exception):
    """Levée lorsqu'une limite de course est atteinte."""
    pass


# ============================================================
# CLASSE MOTEUR
# ============================================================

class MotorController:

    def __init__(
        self,
        pi,
        pin_step,
        pin_dir,
        pin_enable,
        pin_limit_low,
        pin_limit_high,
        resolution,
        v_max,
        a_max,
        nom,
        offset_homing
    ):

        self.pi = pi

        self.pin_step = pin_step
        self.pin_dir = pin_dir
        self.pin_enable = pin_enable

        self.pin_limit_low = pin_limit_low
        self.pin_limit_high = pin_limit_high

        self.resolution = resolution

        self.v_max = v_max
        self.a_max = a_max

        self.nom = nom
        self.offset_homing = offset_homing

        # Position interne de l'axe
        self.position = 0.0

        # Configuration GPIO
        self.pi.set_mode(
            self.pin_step,
            pigpio.OUTPUT
        )

        self.pi.set_mode(
            self.pin_dir,
            pigpio.OUTPUT
        )

        self.pi.set_mode(
            self.pin_enable,
            pigpio.OUTPUT
        )

        # Capteurs en entrée
        self.pi.set_mode(
            self.pin_limit_low,
            pigpio.INPUT
        )

        self.pi.set_mode(
            self.pin_limit_high,
            pigpio.INPUT
        )

        # Pull-up interne
        self.pi.set_pull_up_down(
            self.pin_limit_low,
            pigpio.PUD_UP
        )

        self.pi.set_pull_up_down(
            self.pin_limit_high,
            pigpio.PUD_UP
        )

        # Désactivation initiale
        self.enable(False)


    # ========================================================
    # ENABLE
    # ========================================================

    def enable(self, actif):

        # ENABLE du TMC2209 est actif à l'état bas
        if actif:
            self.pi.write(
                self.pin_enable,
                0
            )
        else:
            self.pi.write(
                self.pin_enable,
                1
            )


    # ========================================================
    # VERIFICATION LIMITES
    # ========================================================

    def _limite_atteinte(self):

        limite_basse = (
            self.pi.read(
                self.pin_limit_low
            ) == 0
        )

        limite_haute = (
            self.pi.read(
                self.pin_limit_high
            ) == 0
        )

        return limite_basse or limite_haute


    # ========================================================
    # HOMING
    # ========================================================

    def homing(self):

        print(
            f"[{self.nom}] Début du homing..."
        )

        self.enable(True)

        # Vitesse réduite pour le référencement
        vitesse_homing = self.v_max * 0.30

        # Direction vers le capteur bas/zéro
        self.pi.write(
            self.pin_dir,
            0
        )

        # Fréquence approximative des impulsions
        steps_per_sec = (
            vitesse_homing
            / self.resolution
        )

        if steps_per_sec <= 0:
            raise ValueError(
                "Vitesse de homing invalide."
            )

        delay = 1.0 / (
            2.0 * steps_per_sec
        )

        while (
            self.pi.read(
                self.pin_limit_low
            ) != 0
        ):

            self.pi.write(
                self.pin_step,
                1
            )

            time.sleep(delay)

            self.pi.write(
                self.pin_step,
                0
            )

            time.sleep(delay)

        # Position de référence
        self.position = self.offset_homing

        print(
            f"[{self.nom}] Homing terminé : "
            f"{self.position:.2f}"
        )


    # ========================================================
    # PROFIL DE VITESSE
    # ========================================================

    def _profil_trapezoidal(self, distance):

        distance_abs = abs(distance)

        # Nombre de micro-pas
        nombre_steps = max(
            1,
            round(
                distance_abs
                / self.resolution
            )
        )

        # Nombre de pas nécessaires à l'accélération
        steps_acc = int(
            self.v_max**2
            / (
                2.0
                * self.a_max
                * self.resolution
            )
        )

        # Pour éviter que accélération + décélération
        # dépassent la moitié du déplacement
        steps_acc = min(
            steps_acc,
            nombre_steps // 2
        )

        # Cas très court
        if steps_acc == 0:

            delay = (
                1.0
                / (
                    2.0
                    * (
                        self.v_max
                        / self.resolution
                    )
                )
            )

            return [delay] * nombre_steps

        steps_plateau = (
            nombre_steps
            - 2 * steps_acc
        )

        delais = []

        # ----------------------------------------------------
        # Accélération
        # ----------------------------------------------------

        for i in range(1, steps_acc + 1):

            vitesse = math.sqrt(
                2.0
                * self.a_max
                * i
                * self.resolution
            )

            vitesse = min(
                vitesse,
                self.v_max
            )

            delais.append(
                1.0
                / (2.0 * vitesse / self.resolution)
            )

        # ----------------------------------------------------
        # Vitesse constante
        # ----------------------------------------------------

        vitesse = self.v_max

        delay_plateau = (
            1.0
            / (
                2.0
                * vitesse
                / self.resolution
            )
        )

        delais.extend(
            [delay_plateau]
            * steps_plateau
        )

        # ----------------------------------------------------
        # Décélération
        # ----------------------------------------------------

        for i in range(
            steps_acc,
            0,
            -1
        ):

            vitesse = math.sqrt(
                2.0
                * self.a_max
                * i
                * self.resolution
            )

            vitesse = min(
                vitesse,
                self.v_max
            )

            delais.append(
                1.0
                / (
                    2.0
                    * vitesse
                    / self.resolution
                )
            )

        return delais


    # ========================================================
    # DEPLACEMENT
    # ========================================================

    def aller_a(self, position_cible):

        distance = (
            position_cible
            - self.position
        )

        if abs(distance) < (
            self.resolution / 2
        ):
            return

        # Direction
        direction = 1 if distance > 0 else 0

        # Génération du profil
        delais = self._profil_trapezoidal(
            distance
        )

        self.enable(True)

        self.pi.write(
            self.pin_dir,
            direction
        )

        # Exécution des impulsions
        for delay in delais:

            # Vérification sécurité
            if self._limite_atteinte():

                self.enable(False)

                raise LimiteCourseError(
                    f"[{self.nom}] "
                    "Limite de course atteinte."
                )

            # STEP HIGH
            self.pi.write(
                self.pin_step,
                1
            )

            time.sleep(delay)

            # STEP LOW
            self.pi.write(
                self.pin_step,
                0
            )

            time.sleep(delay)

            # Mise à jour position
            if direction == 1:
                self.position += (
                    self.resolution
                )
            else:
                self.position -= (
                    self.resolution
                )

        # Correction de l'erreur numérique
        self.position = position_cible

        print(
            f"[{self.nom}] "
            f"Position = {self.position:.2f}"
        )


# ============================================================
# CREATION DES DEUX AXES
# ============================================================

def creer_axes(pi):

    axe_z = MotorController(
        pi=pi,
        pin_step=config.PIN_Z_STEP,
        pin_dir=config.PIN_Z_DIR,
        pin_enable=config.PIN_Z_ENABLE,
        pin_limit_low=config.PIN_Z_LIMIT_BAS,
        pin_limit_high=config.PIN_Z_LIMIT_HAUT,
        resolution=config.RESOLUTION_Z_MM,
        v_max=config.V_MAX_Z_MM_S,
        a_max=config.A_MAX_Z_MM_S2,
        nom="AXE Z",
        offset_homing=config.OFFSET_HOMING_Z_MM
    )

    axe_theta = MotorController(
        pi=pi,
        pin_step=config.PIN_T_STEP,
        pin_dir=config.PIN_T_DIR,
        pin_enable=config.PIN_T_ENABLE,
        pin_limit_low=config.PIN_T_ZERO,
        pin_limit_high=config.PIN_T_LIMIT_ROTATION,
        resolution=config.RESOLUTION_T_DEG,
        v_max=config.V_MAX_T_DEG_S,
        a_max=config.A_MAX_T_DEG_S2,
        nom="AXE THETA",
        offset_homing=config.OFFSET_HOMING_T_DEG
    )

    return axe_z, axe_theta
