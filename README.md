# FS-i6X BLE — pont iBUS vers manette Bluetooth

Transforme un récepteur FlySky (sortie i-BUS) + un ESP32 en manette Bluetooth LE
reconnue par Windows, pour voler dans VelociDrone avec la FS-i6X, sans fil.

```
FS-i6X  ))) 2,4 GHz )))  Récepteur FlySky --iBUS-->  ESP32  ))) BLE )))  PC / VelociDrone
```

## Branchement

| Récepteur (port i-BUS « SERVO ») | ESP32 (esp32dev) |
|---|---|
| Signal (S) | GPIO16 |
| + | 5V / VIN |
| − | GND |

Respecte l'ordre S, +, − imprimé sur le récepteur. Pour un C3 ou un S3, la broche
se change dans `platformio.ini` (`IBUS_RX_PIN`).

## Radio

1. Associe le récepteur à la FS-i6X.
2. System → RX setup → Output mode → **Serial: i-BUS**.
3. Assigne les interrupteurs aux voies 5 et 6 (Functions setup → Aux. channels).

## Compiler et téléverser

1. Ouvre le dossier dans VS Code ; accepte l'extension PlatformIO recommandée.
2. Choisis l'environnement (`esp32dev`, `esp32c3` ou `esp32s3`) dans la barre d'état.
3. **Upload**, puis **Monitor** (115200 bauds).

Pour voir les voies défiler dans le moniteur série, mets `DEBUG_CHANNELS=1`
dans `platformio.ini`.

## Côté PC

1. Paramètres Windows → Bluetooth → Ajouter un appareil → **FS-i6X BLE**.
2. Vérifie les axes dans `joy.cpl` (Contrôleurs de jeu).
3. VelociDrone → Settings → Controller : choisis la manette, **calibre**, puis
   assigne Throttle / Yaw / Pitch / Roll / Arm.

## Correspondance des voies

| Voie iBUS | Rôle (mode 2) | Axe HID |
|---|---|---|
| 1 | Aileron (roll) | X |
| 2 | Profondeur (pitch) | Y |
| 3 | Gaz | Z |
| 4 | Lacet (yaw) | RX |
| 5 | AUX1 (armement) | RY |
| 6 | AUX2 | RZ |
| 7 | AUX3 | Slider 1 |
| 8 | AUX4 | Slider 2 |

Modifiable dans le tableau `CHANNEL_FOR_AXIS` de `src/main.cpp`.

## Perte de signal

Sans trame iBUS pendant 250 ms, les manches reviennent au centre et les gaz en
bas, pour que le simulateur ne garde pas la dernière position.

## Télémétrie : tension de la pile sur la radio

L'ESP32 mesure la pile (une cellule Li-ion, ex. 18650) et se présente au
récepteur comme un capteur de tension externe. La FS-i6X l'affiche (**Ext.V**)
et Windows montre le pourcentage à côté de l'appareil Bluetooth.

Il faut un récepteur avec un port **SENS** (FS-iA6B, FS-iA10B).

| Élément | ESP32-C3 SuperMini |
|---|---|
| Port SENS du récepteur, signal (S) | GPIO6 (direct) |
| Port SENS du récepteur, signal (S) | GPIO7 via une diode 1N4148, **bague côté GPIO7** |
| Port SENS, − | GND |
| Pile + → 100 kΩ → GPIO3 → 100 kΩ → GND | pont diviseur (÷2) |

Le capteur répond dès le démarrage : alimente l'ESP32 et le récepteur
ensemble. Si **Ext.V** n'apparaît pas, redémarre le récepteur. La ligne d'état
du moniteur série affiche `pile:` et le nombre de `requetes` reçues du récepteur.
Broches et rapport du pont dans `platformio.ini` (`SENS_*`, `BATT_*`).

## Dépannage

| Symptôme | Piste |
|---|---|
| Moniteur affiche `--` | Pas d'iBUS : sortie radio pas en i-BUS, mauvaise broche ou fil sur le mauvais port |
| `RX` mais `BLE:off` | Appairage Windows ; supprime l'appareil et réassocie après un reflash |
| Axes inversés | Inverse la voie dans la radio (Reverse) ou dans VelociDrone |
| ESP32-S2 | Pas de Bluetooth : il faut un ESP32 classique, C3 ou S3 |
| `telemetrie:0 requetes` | Fil sur le port SERVO au lieu de SENS, ou récepteur démarré avant l'ESP32 |
| Requêtes reçues mais pas d'Ext.V | Diode inversée : la bague doit être côté TX de l'ESP32 |
| Tension fausse | Ajuste `BATT_DIVIDER` à (R1 + R2) / R2 |
