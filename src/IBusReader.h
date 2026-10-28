// Décodeur iBUS minimal, sans timer ni dépendance.
//
// Trame FlySky iBUS (115200 8N1), 32 octets :
//   [0]    0x20  longueur
//   [1]    0x40  commande (données servo)
//   [2..29] 14 voies, uint16 little-endian (1000..2000)
//   [30..31] checksum = 0xFFFF - somme des octets 0..29, little-endian
#pragma once
#include <Arduino.h>

class IBusReader {
public:
  static const uint8_t CHANNELS = 14;

  void begin(HardwareSerial& serial, int8_t rxPin, int8_t txPin = -1) {
    _serial = &serial;
    _serial->begin(115200, SERIAL_8N1, rxPin, txPin);
    for (uint8_t i = 0; i < CHANNELS; i++) _ch[i] = 1500;
  }

  // À appeler à chaque tour de loop(). Retourne true si une trame valide a été reçue.
  bool update() {
    bool gotFrame = false;
    while (_serial && _serial->available()) {
      uint8_t b = (uint8_t)_serial->read();

      if (_pos == 0 && b != 0x20) continue;            // attend l'octet de longueur
      if (_pos == 1 && b != 0x40) { _pos = 0; continue; } // pas une trame servo

      _buf[_pos++] = b;
      if (_pos < FRAME_LEN) continue;
      _pos = 0;

      uint16_t sum = 0xFFFF;
      for (uint8_t i = 0; i < FRAME_LEN - 2; i++) sum -= _buf[i];
      uint16_t rxSum = _buf[30] | (_buf[31] << 8);
      if (sum != rxSum) { _errors++; continue; }

      for (uint8_t i = 0; i < CHANNELS; i++) {
        _ch[i] = _buf[2 + i * 2] | ((_buf[3 + i * 2] & 0x0F) << 8);
      }
      _frames++;
      _lastFrameMs = millis();
      gotFrame = true;
    }
    return gotFrame;
  }

  uint16_t channel(uint8_t i) const { return i < CHANNELS ? _ch[i] : 1500; }
  uint32_t lastFrameMs() const { return _lastFrameMs; }
  uint32_t frames() const { return _frames; }
  uint32_t errors() const { return _errors; }

private:
  static const uint8_t FRAME_LEN = 32;
  HardwareSerial* _serial = nullptr;
  uint8_t  _buf[FRAME_LEN];
  uint8_t  _pos = 0;
  uint16_t _ch[CHANNELS];
  uint32_t _lastFrameMs = 0;
  uint32_t _frames = 0;
  uint32_t _errors = 0;
};
