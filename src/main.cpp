// Pont iBUS -> manette BLE pour simulateur (VelociDrone, Liftoff, etc.)
//
// Récepteur FlySky (port i-BUS « SERVO ») -> ESP32 -> PC en Bluetooth LE.
// L'ESP32 s'annonce comme une manette HID standard : aucun pilote requis.

#include <Arduino.h>
#include "IBusReader.h"
#include "IBusSensor.h"
#include <BleGamepad.h>
#include <NimBLEDevice.h>
#include <esp_system.h>

#ifndef IBUS_RX_PIN
#define IBUS_RX_PIN 16
#endif
#ifndef IBUS_TX_PIN
#define IBUS_TX_PIN 17
#endif
#ifndef DEBUG_CHANNELS
#define DEBUG_CHANNELS 0
#endif
// Télémétrie vers la radio (port SENS du récepteur). -1 = désactivée.
#ifndef SENS_RX_PIN
#define SENS_RX_PIN -1
#endif
#ifndef SENS_TX_PIN
#define SENS_TX_PIN -1
#endif
// Mesure de la pile (pont diviseur vers une entrée ADC). -1 = désactivée.
#ifndef BATT_ADC_PIN
#define BATT_ADC_PIN -1
#endif
#ifndef BATT_DIVIDER
#define BATT_DIVIDER 2.0f   // (R1 + R2) / R2 ; 2.0 pour deux résistances égales
#endif

// Le C3 n'a que deux UART : Serial0 est libre puisque Serial passe par l'USB.
#if SOC_UART_NUM > 2
#define SENS_SERIAL Serial2
#else
#define SENS_SERIAL Serial0
#endif

// --- Réglages ---------------------------------------------------------------
static const char*    DEVICE_NAME     = "FS-i6X BLE";
static const char*    MANUFACTURER    = "Fred";
static const uint32_t REPORT_PERIOD   = 5;    // ms entre deux rapports (~200 Hz)
static const uint32_t SIGNAL_TIMEOUT  = 250;  // ms sans trame iBUS = perte de signal
static const int16_t  AXIS_MIN        = 1000; // plage iBUS native
static const int16_t  AXIS_MAX        = 2000;
static const int16_t  AXIS_MID        = 1500;

// Affectation conseillée sur l'émetteur (Functions setup -> Aux. channels) :
//   ch5 SWA, ch6 SWB, ch7 SWC (3 positions), ch8 SWD, ch9 VrA, ch10 VrB.
// Les sticks (ch1..ch4) sont fixes. FlySky mode 2 : ch1 aileron,
// ch2 profondeur, ch3 gaz, ch4 lacet.

// Correspondance voie iBUS (0 = voie 1) -> axe HID. Les 10 voies sont des axes.
enum Axis { AX_X, AX_Y, AX_Z, AX_RX, AX_RY, AX_RZ, AX_SL1, AX_SL2,
            AX_RUDDER, AX_THROTTLE, AX_COUNT };
static const uint8_t CHANNEL_FOR_AXIS[AX_COUNT] = {
  0, // X        <- ch1 aileron
  1, // Y        <- ch2 profondeur
  2, // Z        <- ch3 gaz
  3, // RX       <- ch4 lacet
  4, // RY       <- ch5 SWA
  5, // RZ       <- ch6 SWB
  6, // Slider1  <- ch7 SWC
  7, // Slider2  <- ch8 SWD
  8, // Rudder   <- ch9 VrA
  9, // Throttle <- ch10 VrB
};

// Boutons : chaque position utile des interrupteurs devient un bouton, plus
// simple à affecter dans un simulateur qu'un axe (armement, mode de vol...).
// Android exige d'ailleurs au moins un bouton pour reconnaître une manette.
// Un bouton est appuyé quand la voie est dans [lo, hi].
struct ButtonMap { uint8_t ch; int16_t lo; int16_t hi; };
static const ButtonMap BUTTONS[] = {
  { 4, 1750, 2100 }, // B1 <- SWA actif
  { 5, 1750, 2100 }, // B2 <- SWB actif
  { 6,  900, 1250 }, // B3 <- SWC position 1
  { 6, 1250, 1750 }, // B4 <- SWC position 2 (milieu)
  { 6, 1750, 2100 }, // B5 <- SWC position 3
  { 7, 1750, 2100 }, // B6 <- SWD actif
};
static const uint8_t BUTTON_COUNT = sizeof(BUTTONS) / sizeof(BUTTONS[0]);
// ----------------------------------------------------------------------------

// --- Journal BLE détaillé ------------------------------------------------------
// Chaque événement Bluetooth est affiché dans le moniteur série, avec l'état de
// la liaison. Les codes 0x2xx sont des codes HCI (0x200 + code Bluetooth),
// les codes 0x5xx des erreurs d'appairage (0x500 + code SM).

static uint32_t bleConnectMs  = 0;
static uint32_t bleNotifyOk   = 0;
static uint32_t bleNotifyFail = 0;

static const char* disconnectReasonText(int r) {
  switch (r) {
    case 0x208: return "perte de liaison radio (supervision timeout)";
    case 0x213: return "fermee par le telephone/tablette";
    case 0x216: return "fermee par l'ESP32";
    case 0x205: return "echec d'authentification";
    case 0x206: return "cle d'appairage absente (PIN/key missing)";
    case 0x21a: return "fonction non supportee par l'autre appareil";
    case 0x222: return "timeout du protocole LL";
    case 0x223: return "procedures LL en collision";
    case 0x23b: return "parametres de connexion refuses";
    case 0x23d: return "echec d'integrite (MIC) - cle de chiffrement invalide";
    case 0x23e: return "echec d'etablissement de la connexion";
  }
  if (r >= 0x500 && r < 0x600) return "echec d'appairage (Security Manager)";
  return NimBLEUtils::returnCodeToString(r);
}

static void printConnDesc(const ble_gap_conn_desc& d) {
  const uint8_t* a = d.peer_ota_addr.val;
  Serial.printf("      pair=%02x:%02x:%02x:%02x:%02x:%02x handle=%u"
                " intervalle=%.2fms latence=%u timeout=%ums\n",
                a[5], a[4], a[3], a[2], a[1], a[0], d.conn_handle,
                d.conn_itvl * 1.25f, d.conn_latency, d.supervision_timeout * 10);
  Serial.printf("      chiffre=%d authentifie=%d appaire(bonded)=%d cle=%u octets\n",
                d.sec_state.encrypted, d.sec_state.authenticated,
                d.sec_state.bonded, d.sec_state.key_size);
}

static void printConn(uint16_t handle) {
  ble_gap_conn_desc d;
  if (ble_gap_conn_find(handle, &d) == 0) printConnDesc(d);
}

static int logGapEvent(ble_gap_event* e, void*) {
  const uint32_t t = millis();
  switch (e->type) {
    case BLE_GAP_EVENT_CONNECT:
      Serial.printf("[BLE %lu] CONNEXION status=%d %s\n", (unsigned long)t,
                    e->connect.status, NimBLEUtils::returnCodeToString(e->connect.status));
      if (e->connect.status == 0) {
        bleConnectMs = t;
        bleNotifyOk = bleNotifyFail = 0;
        printConn(e->connect.conn_handle);
      }
      break;

    case BLE_GAP_EVENT_DISCONNECT:
      Serial.printf("[BLE %lu] DECONNEXION raison=0x%03x (%s)\n", (unsigned long)t,
                    e->disconnect.reason, disconnectReasonText(e->disconnect.reason));
      Serial.printf("      duree=%lums rapports envoyes=%lu echecs=%lu\n",
                    (unsigned long)(t - bleConnectMs),
                    (unsigned long)bleNotifyOk, (unsigned long)bleNotifyFail);
      printConnDesc(e->disconnect.conn);
      break;

    case BLE_GAP_EVENT_ENC_CHANGE:
      Serial.printf("[BLE %lu] CHIFFREMENT status=0x%03x %s\n", (unsigned long)t,
                    e->enc_change.status,
                    e->enc_change.status == 0 ? "OK" : disconnectReasonText(e->enc_change.status));
      printConn(e->enc_change.conn_handle);
      break;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
      Serial.printf("[BLE %lu] RE-APPAIRAGE (ancienne cle effacee)\n", (unsigned long)t);
      break;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
      Serial.printf("[BLE %lu] DEMANDE DE CODE action=%u\n", (unsigned long)t,
                    e->passkey.params.action);
      break;

    case BLE_GAP_EVENT_CONN_UPDATE:
      Serial.printf("[BLE %lu] MAJ PARAMETRES status=0x%03x\n", (unsigned long)t,
                    e->conn_update.status);
      printConn(e->conn_update.conn_handle);
      break;

    case BLE_GAP_EVENT_CONN_UPDATE_REQ:
      Serial.printf("[BLE %lu] DEMANDE PARAMETRES intervalle %.2f-%.2fms latence=%u timeout=%ums\n",
                    (unsigned long)t,
                    e->conn_update_req.peer_params->itvl_min * 1.25f,
                    e->conn_update_req.peer_params->itvl_max * 1.25f,
                    e->conn_update_req.peer_params->latency,
                    e->conn_update_req.peer_params->supervision_timeout * 10);
      break;

    case BLE_GAP_EVENT_MTU:
      Serial.printf("[BLE %lu] MTU=%u\n", (unsigned long)t, e->mtu.value);
      break;

    case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
      Serial.printf("[BLE %lu] PHY status=%d tx=%u rx=%u\n", (unsigned long)t,
                    e->phy_updated.status, e->phy_updated.tx_phy, e->phy_updated.rx_phy);
      break;

    case BLE_GAP_EVENT_SUBSCRIBE:
      Serial.printf("[BLE %lu] ABONNEMENT attr=%u notify=%u indicate=%u (raison=%u)\n",
                    (unsigned long)t, e->subscribe.attr_handle, e->subscribe.cur_notify,
                    e->subscribe.cur_indicate, e->subscribe.reason);
      break;

    case BLE_GAP_EVENT_NOTIFY_TX:
      // Trop fréquent pour être affiché : on compte, et on n'affiche que la 1re erreur.
      if (e->notify_tx.status == 0) {
        bleNotifyOk++;
      } else if (bleNotifyFail++ == 0) {
        Serial.printf("[BLE %lu] ECHEC ENVOI RAPPORT status=%d %s\n", (unsigned long)t,
                      e->notify_tx.status, NimBLEUtils::returnCodeToString(e->notify_tx.status));
      }
      break;
  }
  return 0;
}

static const char* resetReasonText(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "mise sous tension";
    case ESP_RST_SW:       return "redemarrage logiciel";
    case ESP_RST_PANIC:    return "PLANTAGE (panic/exception)";
    case ESP_RST_INT_WDT:  return "PLANTAGE (watchdog interruptions)";
    case ESP_RST_TASK_WDT: return "PLANTAGE (watchdog tache)";
    case ESP_RST_WDT:      return "PLANTAGE (watchdog)";
    case ESP_RST_BROWNOUT: return "CHUTE DE TENSION (brownout)";
    case ESP_RST_DEEPSLEEP:return "reveil deep-sleep";
    case ESP_RST_EXT:      return "bouton reset";
    default:               return "autre";
  }
}
// ----------------------------------------------------------------------------

class LoggingGamepad : public BleGamepad {
public:
  using BleGamepad::BleGamepad;
protected:
  void onStarted(NimBLEServer*) override {
    NimBLEDevice::setCustomGapHandler(logGapEvent, nullptr);
  }
};

IBusReader ibus;
IBusSensor sensor;
LoggingGamepad gamepad(DEVICE_NAME, MANUFACTURER, 100);

static uint32_t lastReportMs   = 0;
static uint16_t battMv         = 0;   // tension pile filtrée, 0 = pas de mesure

// Lecture de la pile toutes les 100 ms, lissée (moyenne glissante sur ~8 mesures).
static void updateBattery(uint32_t now) {
#if BATT_ADC_PIN >= 0
  static uint32_t last = 0;
  if (battMv && now - last < 100) return;
  last = now;
  const uint16_t mv = analogReadMilliVolts(BATT_ADC_PIN) * BATT_DIVIDER;
  battMv = battMv ? (battMv * 7 + mv) / 8 : mv;
  sensor.setValue(battMv / 10);
#endif
}

// Courbe de décharge approximative d'une cellule Li-ion (18650, LiPo 1S).
static uint8_t batteryPercent(uint16_t mv) {
  static const uint16_t MV[]  = { 3300, 3500, 3600, 3700, 3800, 3900, 4000, 4100, 4200 };
  static const uint8_t  PCT[] = {    0,    5,   15,   30,   50,   65,   80,   90,  100 };
  if (mv <= MV[0]) return 0;
  for (uint8_t i = 1; i < sizeof(MV) / sizeof(MV[0]); i++) {
    if (mv < MV[i]) return PCT[i - 1] + (PCT[i] - PCT[i - 1]) * (mv - MV[i - 1]) / (MV[i] - MV[i - 1]);
  }
  return 100;
}

static int16_t clampAxis(uint16_t v) {
  if (v < AXIS_MIN) return AXIS_MIN;
  if (v > AXIS_MAX) return AXIS_MAX;
  return (int16_t)v;
}

// Sans signal : manches au centre et gaz en bas, pour que le simulateur
// ne garde pas la dernière position (même logique qu'un failsafe).
static int16_t failsafeValue(uint8_t axis) {
  return (axis == AX_Z) ? AXIS_MIN : AXIS_MID;
}

void setup() {
  // Le capteur démarre en premier : le récepteur cherche ses capteurs dès sa
  // mise sous tension, il faut déjà pouvoir lui répondre.
#if BATT_ADC_PIN >= 0
  analogSetPinAttenuation(BATT_ADC_PIN, ADC_11db);
  updateBattery(millis());
#endif
#if SENS_RX_PIN >= 0
  sensor.begin(SENS_SERIAL, SENS_RX_PIN, SENS_TX_PIN, IBusSensor::TYPE_EXT_VOLTAGE);
#endif

  Serial.begin(115200);
  // Sur USB natif (C3/S3), laisse le temps au moniteur de se connecter.
  for (uint32_t t0 = millis(); !Serial && millis() - t0 < 2000;) {
    sensor.update();
    delay(1);
  }
  Serial.printf("\n=== Demarrage, cause du dernier reset : %s ===\n",
                resetReasonText(esp_reset_reason()));

  ibus.begin(Serial1, IBUS_RX_PIN, IBUS_TX_PIN);

  BleGamepadConfiguration cfg;
  cfg.setControllerType(CONTROLLER_TYPE_JOYSTICK);
  cfg.setAutoReport(false);
  cfg.setButtonCount(BUTTON_COUNT);
  cfg.setHatSwitchCount(0);
  cfg.setWhichAxes(true, true, true, true, true, true, true, true);
  cfg.setAxesMin(AXIS_MIN);
  cfg.setAxesMax(AXIS_MAX);
  cfg.setWhichSimulationControls(true, true, false, false, false); // rudder, throttle
  cfg.setSimulationMin(AXIS_MIN);
  cfg.setSimulationMax(AXIS_MAX);
  gamepad.begin(&cfg);

  Serial.printf("iBUS sur RX=%d, en attente de connexion BLE \"%s\"...\n",
                IBUS_RX_PIN, DEVICE_NAME);
}

void loop() {
  ibus.update();
  sensor.update();

  const uint32_t now = millis();
  updateBattery(now);
  const bool signalOk = ibus.frames() > 0 &&
                        (now - ibus.lastFrameMs()) < SIGNAL_TIMEOUT;

  // La bibliothèque ne considère la manette « connectée » qu'une fois la liaison
  // chiffrée : on trace ce passage, distinct de la connexion radio.
  static bool wasConnected = false;
  if (gamepad.isConnected() != wasConnected) {
    wasConnected = gamepad.isConnected();
    Serial.printf("[GAMEPAD %lu] %s\n", (unsigned long)now,
                  wasConnected ? "pret, envoi des rapports" : "plus d'hote, envoi arrete");
  }

  // Ligne d'état périodique, pour diagnostiquer sans DEBUG_CHANNELS.
  static uint32_t lastStatus = 0;
  if (now - lastStatus > 2000) {
    lastStatus = now;
    Serial.printf("iBUS:%s (trames:%lu err:%lu)  BLE:%s",
                  signalOk ? "OK" : "aucun signal",
                  (unsigned long)ibus.frames(), (unsigned long)ibus.errors(),
                  gamepad.isConnected() ? "connecte" : "en attente");
#if BATT_ADC_PIN >= 0
    Serial.printf("  pile:%.2fV %u%%", battMv / 1000.0f, batteryPercent(battMv));
#endif
#if SENS_RX_PIN >= 0
    Serial.printf("  telemetrie:%lu requetes", (unsigned long)sensor.polls());
#endif
    Serial.println();
  }

#if BATT_ADC_PIN >= 0
  // Niveau de batterie côté Bluetooth : Windows l'affiche avec l'appareil.
  static uint8_t lastPct = 255;
  const uint8_t pct = batteryPercent(battMv);
  if (gamepad.isConnected() && pct != lastPct) {
    lastPct = pct;
    gamepad.setBatteryLevel(pct);
  }
#endif

  if (now - lastReportMs < REPORT_PERIOD) return;
  lastReportMs = now;

  int16_t v[AX_COUNT];
  for (uint8_t a = 0; a < AX_COUNT; a++) {
    v[a] = signalOk ? clampAxis(ibus.channel(CHANNEL_FOR_AXIS[a]))
                    : failsafeValue(a);
  }

#if DEBUG_CHANNELS
  static uint32_t lastPrint = 0;
  if (now - lastPrint > 200) {
    lastPrint = now;
    Serial.printf("%s BLE:%s  ", signalOk ? "RX" : "--",
                  gamepad.isConnected() ? "on " : "off");
    for (uint8_t a = 0; a < AX_COUNT; a++) Serial.printf("%4d ", v[a]);
    Serial.printf(" trames:%lu err:%lu\n",
                  (unsigned long)ibus.frames(), (unsigned long)ibus.errors());
  }
#endif

  if (!gamepad.isConnected()) return;

  gamepad.setX(v[AX_X]);
  gamepad.setY(v[AX_Y]);
  gamepad.setZ(v[AX_Z]);
  gamepad.setRX(v[AX_RX]);
  gamepad.setRY(v[AX_RY]);
  gamepad.setRZ(v[AX_RZ]);
  gamepad.setSlider1(v[AX_SL1]);
  gamepad.setSlider2(v[AX_SL2]);
  gamepad.setRudder(v[AX_RUDDER]);
  gamepad.setThrottle(v[AX_THROTTLE]);
  for (uint8_t b = 0; b < BUTTON_COUNT; b++) {
    const uint16_t c = ibus.channel(BUTTONS[b].ch);
    const bool on = signalOk && c >= BUTTONS[b].lo && c < BUTTONS[b].hi;
    if (on) gamepad.press(BUTTON_1 + b);
    else    gamepad.release(BUTTON_1 + b);
  }
  gamepad.sendReport();
}
