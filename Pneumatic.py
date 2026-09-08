"""
pneumatic.py
Contrôle de l'électrovanne de préhension via le relais SSR, lecture du
capteur de pression NPN, et gestion des temporisations de mise en
dépression (préhension par ventouse silicone).
"""
 
import time
import pigpio
 
import config
 
 
class DepressionError(Exception):
    """Levée quand la dépression n'a pas pu être confirmée après les essais autorisés."""
 
 
class SystemePneumatique:
    def __init__(self, pi: pigpio.pi):
        self.pi = pi
        self.pi.set_mode(config.PIN_VANNE_SSR, pigpio.OUTPUT)
        self.pi.set_mode(config.PIN_CAPTEUR_PRESSION, pigpio.INPUT)
        self.pi.set_pull_up_down(config.PIN_CAPTEUR_PRESSION, pigpio.PUD_UP)
        self.vanne_active = False
        self.desactiver_vanne()
 
    # ------------------------------------------------------------------ #
    def activer_vanne(self):
        self.pi.write(config.PIN_VANNE_SSR, 1)
        self.vanne_active = True
 
    def desactiver_vanne(self):
        self.pi.write(config.PIN_VANNE_SSR, 0)
        self.vanne_active = False
 
    # ------------------------------------------------------------------ #
    def depression_detectee(self) -> bool:
        # capteur de pression NPN : état bas = vide confirmé
        return self.pi.read(config.PIN_CAPTEUR_PRESSION) == 0
 
    # ------------------------------------------------------------------ #
    def prise_avec_verification(self):
        """
        Active l'électrovanne et vérifie la dépression.
        Jusqu'à VACUUM_ESSAIS_MAX essais, chacun borné par un timeout de
        VACUUM_TIMEOUT_S secondes (cf. état VACUUM de la FSM, Tableau 39).
        Lève DepressionError si la dépression n'est confirmée par
        aucun essai.
        """
        self.activer_vanne()
 
        for essai in range(1, config.VACUUM_ESSAIS_MAX + 1):
            t0 = time.time()
            while time.time() - t0 < config.VACUUM_TIMEOUT_S:
                if self.depression_detectee():
                    return True
                time.sleep(0.02)
            # essai suivant : petite coupure/relance de la vanne avant de retenter
            self.desactiver_vanne()
            time.sleep(0.05)
            self.activer_vanne()
 
        self.desactiver_vanne()
        raise DepressionError(
            f"Dépression non confirmée après {config.VACUUM_ESSAIS_MAX} essais."
        )
 
    # ------------------------------------------------------------------ #
    def relacher(self):
        """Coupe l'électrovanne pour relâcher la pièce (état RELEASE)."""
        self.desactiver_vanne()