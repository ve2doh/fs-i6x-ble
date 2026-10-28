// Capteur de télémétrie iBUS : l'ESP32 se fait passer pour un capteur FlySky
// branché sur le port « SENS » du récepteur, qui relaie la valeur à la radio.
//
// Bus half-duplex sur un seul fil (115200 8N1). Le récepteur interroge chaque
// adresse avec une requête de 4 octets :
//   [0]    0x04  longueur
//   [1]    commande | adresse (1..15)
//   [2..3] checksum = 0xFFFF - somme des octets précédents, little-endian
// Commandes : 0x80 découverte (on renvoie la requête telle quelle)
//             0x90 type      (réponse : type, taille des données = 2)
//             0xA0 mesure    (réponse : valeur uint16 little-endian)
//
// Branchement : RX direct sur le fil signal, TX à travers une diode (bague côté
// TX). On relit donc nos propres octets sur RX : ils sont comptés et ignorés.
#pragma once
#include <Arduino.h>

class IBusSensor {
public:
  static const uint8_t TYPE_EXT_VOLTAGE = 0x03; // valeur en centièmes de volt

  void begin(HardwareSerial& serial, int8_t rxPin, int8_t txPin, uint8_t type) {
    _serial = &serial;
    _type = type;
    _serial->begin(115200, SERIAL_8N1, rxPin, txPin);
  }

  void setValue(uint16_t v) { _value = v; }

  // À appeler souvent : le récepteur attend la réponse dans les millisecondes qui suivent.
  void update() {
    // L'écho arrive pendant l'envoi : passé ce délai, il n'y en aura pas.
    if (_echo > 0 && millis() - _replyMs > 5) _echo = 0;

    while (_serial && _serial->available()) {
      uint8_t b = (uint8_t)_serial->read();
      if (_echo > 0) { _echo--; continue; }            // écho de notre réponse

      // Fenêtre glissante sur les 4 derniers octets : se resynchronise seule.
      _buf[0] = _buf[1]; _buf[1] = _buf[2]; _buf[2] = _buf[3]; _buf[3] = b;
      if (_buf[0] != 0x04) continue;
      if (checksum(_buf, 2) != (uint16_t)(_buf[2] | (_buf[3] << 8))) continue;
      if ((_buf[1] & 0x0F) != ADDRESS) { _buf[0] = 0; continue; }

      const uint8_t cmd = _buf[1];
      _buf[0] = 0;                                      // requête consommée
      switch (cmd & 0xF0) {
        case 0x80: reply(cmd, nullptr, 0); break;
        case 0x90: { const uint8_t d[] = { _type, 0x02 }; reply(cmd, d, 2); break; }
        case 0xA0: {
          const uint8_t d[] = { (uint8_t)_value, (uint8_t)(_value >> 8) };
          reply(cmd, d, 2);
          _polls++;
          break;
        }
      }
    }
  }

  uint32_t polls() const { return _polls; }  // mesures demandées par le récepteur

private:
  static const uint8_t ADDRESS = 1;          // premier capteur externe

  static uint16_t checksum(const uint8_t* p, uint8_t n) {
    uint16_t sum = 0xFFFF;
    for (uint8_t i = 0; i < n; i++) sum -= p[i];
    return sum;
  }

  void reply(uint8_t cmd, const uint8_t* data, uint8_t n) {
    uint8_t out[6];
    const uint8_t len = n + 4;
    out[0] = len;
    out[1] = cmd;
    for (uint8_t i = 0; i < n; i++) out[2 + i] = data[i];
    const uint16_t sum = checksum(out, len - 2);
    out[len - 2] = sum & 0xFF;
    out[len - 1] = sum >> 8;
    _serial->write(out, len);
    _echo += len;
    _replyMs = millis();
  }

  HardwareSerial* _serial = nullptr;
  uint8_t  _buf[4] = {};
  uint8_t  _type = TYPE_EXT_VOLTAGE;
  uint16_t _value = 0;
  uint8_t  _echo = 0;
  uint32_t _replyMs = 0;
  uint32_t _polls = 0;
};
