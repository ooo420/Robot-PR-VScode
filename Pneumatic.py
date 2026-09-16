"""
pneumatic.py

Gestion du système de préhension par vide :
- activation de la vanne
- désactivation de la vanne
- lecture du capteur de pression
- vérification de la prise LCD
"""

import time
import pigpio
import config


# ============================================================
# EXCEPTION
# ============================================================

class DepressionError(Exception):
    """Levée lorsqu'une dépression suffisante n'est pas détectée."""
    pass


# ============================================================
# SYSTEME PNEUMATIQUE
# ============================================================

class SystemePneumatique:

    def __init__(self, pi):

        self.pi = pi

        # Configuration vanne
        self.pi.set_mode(
            config.PIN_VANNE_SSR,
            pigpio.OUTPUT
        )

        # Configuration capteur
        self.pi.set_mode(
            config.PIN_CAPTEUR_PRESSION,
            pigpio.INPUT
        )

        self.pi.set_pull_up_down(
            config.PIN_CAPTEUR_PRESSION,
            pigpio.PUD_UP
        )

        # Etat initial
        self.vanne_active = False

        self.desactiver_vanne()


    # ========================================================
    # ACTIVATION VANNE
    # ========================================================

    def activer_vanne(self):

        self.pi.write(
            config.PIN_VANNE_SSR,
            1
        )

        self.vanne_active = True

        print("[PNEUMATIQUE] Vide activé.")


    # ========================================================
    # DESACTIVATION VANNE
    # ========================================================

    def desactiver_vanne(self):

        self.pi.write(
            config.PIN_VANNE_SSR,
            0
        )

        self.vanne_active = False

        print("[PNEUMATIQUE] Vide désactivé.")


    # ========================================================
    # DETECTION DEPRESSION
    # ========================================================

    def depression_detectee(self):

        # Capteur NPN actif à l'état bas
        return (
            self.pi.read(
                config.PIN_CAPTEUR_PRESSION
            ) == 0
        )


    # ========================================================
    # PRISE AVEC VERIFICATION
    # ========================================================

    def prise_avec_verification(self):

        for tentative in range(
            1,
            config.VACUUM_ESSAIS_MAX + 1
        ):

            print(
                f"[PNEUMATIQUE] "
                f"Tentative {tentative}/"
                f"{config.VACUUM_ESSAIS_MAX}"
            )

            self.activer_vanne()

            debut = time.time()

            while (
                time.time() - debut
                < config.VACUUM_TIMEOUT_S
            ):

                if self.depression_detectee():

                    print(
                        "[PNEUMATIQUE] "
                        "Prise LCD confirmée."
                    )

                    return True

                time.sleep(0.02)

            # Echec de la tentative
            self.desactiver_vanne()

            time.sleep(0.05)

        # Toutes les tentatives ont échoué
        self.desactiver_vanne()

        raise DepressionError(
            "Échec de la prise par vide."
        )


    # ========================================================
    # RELACHEMENT
    # ========================================================

    def relacher(self):

        self.desactiver_vanne()

        print(
            "[PNEUMATIQUE] "
            "LCD relâché."
        )
