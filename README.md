# Coil Winder Control

Logiciel de pilotage de la bobineuse (2 moteurs pas à pas) et du générateur de courant (MOSFET + INA219).

| Dossier / fichier | Rôle |
|---|---|
| `main.py` | Interface (Tkinter + pyserial) |
| `bobin_v0/` | Firmware bobineuse (moteurs) |
| `current/` | Firmware générateur de courant (régulation PID) |
| `firmware/*.hex` | Firmwares précompilés (Uno / Nano), embarqués dans l'exe |
| `build_exe.bat` | Construit `dist\CoilWinder.exe` (portable, un seul fichier) |

## Lancer

```
pip install -r requirements.txt
python main.py
```

## Onglets

- **Bobineuse** : contrôle manuel des 2 moteurs, bobinage automatique (pause / reprise / annulation),
  progression avec temps restant, préréglages de bobinage enregistrables.
- **Courant** : arrêt / régulation (consigne en mA) / PWM manuel, réglage du PID (Kp, Ki, Kd, période,
  filtre, limite de surintensité, PWM 31 kHz), sauvegarde EEPROM, mesures en direct, courbe temps réel
  (consigne, courant brut, filtré, PWM), curseur au survol, pause, export CSV et enregistrement continu.
- **Firmware** : téléverse `bobin_v0` ou `current` (ou n'importe quel `.ino` / `.hex`) sur l'Arduino.
  - *Compiler les sources* nécessite `arduino-cli` (détecté automatiquement dans l'IDE Arduino 2).
    Le bouton « Installer les dépendances » installe le core AVR et la lib Adafruit INA219.
  - *Binaire précompilé* + téléverseur *Intégré* : aucune dépendance (utilisé par l'exe portable).
  - Lancé depuis les sources, chaque compilation pour Uno met à jour `firmware/<sketch>.hex`.

À la connexion, le firmware est identifié (`READY;WINDER` / `READY;CURRENT`) : si ce n'est pas le bon,
l'application propose de téléverser le bon programme.

**Échap** = arrêt d'urgence (moteurs + sortie courant), quel que soit l'onglet affiché.
La sortie courant est aussi coupée à la déconnexion et à la fermeture.

## Protocole série (115200 bauds, une commande par ligne)

Bobineuse : `S;rpm1;rpm2`, `W;fil;spires;couches;rpm;sens`, `P`, `R`, `X`, `?` (voir l'en-tête de `bobin_v0.ino`).

Courant : `C;mA`, `O;pwm`, `X`, `K;kp;ki;kd`, `T;ms`, `A;alpha`, `L;mA`, `F;0|1`, `B;0|1`, `Z`, `G`, `E`, `D`, `?`
(voir l'en-tête de `current.ino`). Un nombre seul (ex. `40`) fixe la consigne comme avant, et la télémétrie
reste lisible par le traceur série de l'IDE Arduino.

## Exe portable

Lancer `build_exe.bat` → `dist\CoilWinder.exe`. Les réglages sont stockés dans
`coil_winder_settings.json` à côté de l'exe (ou dans le dossier utilisateur si ce dossier est en lecture seule).
