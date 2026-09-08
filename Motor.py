"""
motor_control.py
Pilotage des deux axes du robot PR (Z : NEMA 23 / vis à billes G1610,
theta : NEMA 17 / réducteur harmonique PG14-50). Génération d'une
rampe de vitesse trapézoïdale et gestion sécurisée des fins de course
via la bibliothèque pigpio (accès DMA matériel, faible latence).
"""

import time
import pigpio

import config


class LimiteCourseError(Exception):
    """Levée quand un capteur de fin de course est atteint pendant un mouvement."""


class MotorController:
    """
    Contrôleur générique pour un axe piloté par un driver STEP/DIR
    (TMC2209), avec génération d'impulsions par rampe trapézoïdale
    et surveillance des capteurs de fin de course.
    """

    def __init__(self, pi: pigpio.pi, pin_step: int, pin_dir: int, pin_enable: int,
                 pin_limit_low: int, pin_limit_high: int,
                 resolution_per_step: float, v_max: float, a_max: float,
                 nom_axe: str, offset_homing: float = 0.0):
        self.pi = pi
        self.pin_step = pin_step
        self.pin_dir = pin_dir
        self.pin_enable = pin_enable
        self.pin_limit_low = pin_limit_low
        self.pin_limit_high = pin_limit_high
        self.resolution = resolution_per_step   # unité physique par micro-pas
        self.v_max = v_max
        self.a_max = a_max
        self.nom_axe = nom_axe
        # Position réinjectée après homing : le capteur bas ne correspond
        # pas forcément à 0 dans le repère absolu (ex. axe Z : 170 mm).
        self.offset_homing = offset_homing
        self.position = 0.0                      # position courante (mm ou degrés)

        self.pi.set_mode(self.pin_step, pigpio.OUTPUT)
        self.pi.set_mode(self.pin_dir, pigpio.OUTPUT)
        self.pi.set_mode(self.pin_enable, pigpio.OUTPUT)
        self.pi.set_mode(self.pin_limit_low, pigpio.INPUT)
        self.pi.set_mode(self.pin_limit_high, pigpio.INPUT)
        self.pi.set_pull_up_down(self.pin_limit_low, pigpio.PUD_UP)
        self.pi.set_pull_up_down(self.pin_limit_high, pigpio.PUD_UP)

        self.enable(False)

    # ------------------------------------------------------------------ #
    # Activation du driver
    # ------------------------------------------------------------------ #
    def enable(self, actif: bool):
        # drivers TMC2209 : ENABLE actif à l'état bas
        self.pi.write(self.pin_enable, 0 if actif else 1)

    # ------------------------------------------------------------------ #
    # Fins de course
    # ------------------------------------------------------------------ #
    def _limite_atteinte(self) -> bool:
        return self.pi.read(self.pin_limit_low) == 0 or self.pi.read(self.pin_limit_high) == 0

    # ------------------------------------------------------------------ #
    # Homing : recherche du capteur bas (Z) ou du capteur 0° (theta)
    # ------------------------------------------------------------------ #
    def homing(self, sens: int = -1, vitesse: float = None):
        """
        Déplace l'axe vers son capteur de référence (fin de course basse
        pour Z, capteur theta=0° pour theta) puis réinitialise le
        compteur de position à self.offset_homing (170 mm pour l'axe Z,
        conformément à la plage de course [170, 370] mm du rapport ;
        0° pour l'axe theta). Élimine toute dérive accumulée lors du
        cycle précédent.
        """
        vitesse = vitesse or (self.v_max * 0.3)   # homing à vitesse réduite
        self.enable(True)
        self.pi.write(self.pin_dir, 1 if sens > 0 else 0)

        delay = 1.0 / (vitesse / self.resolution) if vitesse > 0 else 0.01
        while self.pi.read(self.pin_limit_low) != 0:
            self.pi.write(self.pin_step, 1)
            time.sleep(delay / 2)
            self.pi.write(self.pin_step, 0)
            time.sleep(delay / 2)

        self.position = self.offset_homing
        return True

    # ------------------------------------------------------------------ #
    # Rampe trapézoïdale : accélération -> vitesse constante -> décélération
    # ------------------------------------------------------------------ #
    def _profil_trapezoidal(self, distance: float):
        """
        Génère la liste des délais inter-impulsions (en secondes) pour
        une rampe trapézoïdale sur la distance donnée (valeur absolue,
        dans l'unité physique de l'axe : mm ou degrés).
        """
        n_pas = max(1, round(abs(distance) / self.resolution))

        # distance (en nombre de pas) nécessaire pour atteindre v_max
        pas_acc = max(1, round((self.v_max ** 2) / (2 * self.a_max) / self.resolution))
        pas_acc = min(pas_acc, n_pas // 2)
        pas_palier = max(0, n_pas - 2 * pas_acc)

        delays = []

        # Phase 1 : accélération linéaire (0 -> v_max)
        for i in range(pas_acc):
            v = max(self.v_max * ((i + 1) / pas_acc) ** 0.5, self.v_max * 0.05)
            delays.append(self.resolution / v)

        # Phase 2 : vitesse constante
        delays.extend([self.resolution / self.v_max] * pas_palier)

        # Phase 3 : décélération linéaire (v_max -> 0)
        for i in range(pas_acc):
            v = max(self.v_max * (1 - (i + 1) / pas_acc) ** 0.5, self.v_max * 0.05)
            delays.append(self.resolution / v)

        return delays

    # ------------------------------------------------------------------ #
    # Déplacement vers une position absolue
    # ------------------------------------------------------------------ #
    def aller_a(self, position_cible: float):
        """
        Déplace l'axe de sa position courante jusqu'à position_cible,
        en suivant une rampe trapézoïdale. Lève LimiteCourseError si un
        capteur de fin de course est atteint pendant le mouvement.
        """
        distance = position_cible - self.position
        sens = 1 if distance >= 0 else -1
        delays = self._profil_trapezoidal(distance)

        self.enable(True)
        self.pi.write(self.pin_dir, 1 if sens > 0 else 0)

        for delay in delays:
            if self._limite_atteinte():
                self.enable(False)
                raise LimiteCourseError(
                    f"Fin de course atteinte sur l'axe {self.nom_axe} pendant le mouvement."
                )
            self.pi.write(self.pin_step, 1)
            time.sleep(delay / 2)
            self.pi.write(self.pin_step, 0)
            time.sleep(delay / 2)
            self.position += sens * self.resolution

        self.position = position_cible
        return self.position


def creer_axes(pi: pigpio.pi):
    """Instancie les deux contrôleurs d'axes à partir de config.py."""
    axe_z = MotorController(
        pi, config.PIN_Z_STEP, config.PIN_Z_DIR, config.PIN_Z_ENABLE,
        config.PIN_Z_LIMIT_BAS, config.PIN_Z_LIMIT_HAUT,
        config.RESOLUTION_Z_MM, config.V_MAX_Z_MM_S, config.A_MAX_Z_MM_S2,
        nom_axe="Z", offset_homing=config.OFFSET_HOMING_Z_MM,
    )
    axe_theta = MotorController(
        pi, config.PIN_T_STEP, config.PIN_T_DIR, config.PIN_T_ENABLE,
        config.PIN_T_ZERO, config.PIN_T_LIMIT_ROTATION,
        config.RESOLUTION_T_DEG, config.V_MAX_T_DEG_S, config.A_MAX_T_DEG_S2,
        nom_axe="theta", offset_homing=config.OFFSET_HOMING_T_DEG,
    )
    return axe_z, axe_theta
