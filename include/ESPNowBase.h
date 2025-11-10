#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "CroLibMotion.h"

// Simple ESP-NOW wrapper for CroMotion Base
class ESPNowBaseController {
public:
  ESPNowBaseController() {
    memset(bcast_mac_, 0xFF, sizeof(bcast_mac_));
  }

  // Init ESP-NOW on fixed channel and add a broadcast peer
  bool begin(uint8_t channel = 1, const uint8_t *pmk16 = nullptr) {
    WiFi.mode(WIFI_STA);

    // Lock WiFi channel
    esp_err_t err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    Serial.printf("[ESPNowBase] set_channel(%u) -> %s (0x%X)\n",
                  channel, esp_err_to_name(err), (unsigned)err);

    // Init ESP-NOW
    err = esp_now_init();
    if (err != ESP_OK) {
      Serial.printf("[ESPNowBase] esp_now_init failed: %s (0x%X)\n",
                    esp_err_to_name(err), (unsigned)err);
      return false;
    }

    if (pmk16) {
      esp_now_set_pmk(pmk16);
    }

    // ----- Add broadcast peer FF:FF:FF:FF:FF:FF -----
    esp_now_peer_info_t peer{};
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, bcast_mac_, 6);
    peer.ifidx   = WIFI_IF_STA;
    peer.channel = channel;
    peer.encrypt = false;

    err = esp_now_add_peer(&peer);
    Serial.printf("[ESPNowBase] add_peer(broadcast) -> %s (0x%X)\n",
                  esp_err_to_name(err), (unsigned)err);

    if (err == ESP_OK || err == ESP_ERR_ESPNOW_EXIST) {
      has_bcast_ = true;
    } else {
      has_bcast_ = false;
      Serial.println("[ESPNowBase] broadcast peer not ready, will fallback to nullptr send");
    }

    // Debug peer table
    esp_now_peer_num_t stats;
    esp_now_get_peer_num(&stats);
    Serial.printf("[ESPNowBase] Peer table: total=%u, encrypted=%u\n",
                  stats.total_num, stats.encrypt_num);

    Serial.printf("[ESPNowBase] Ready on channel %u\n", channel);
    return true;
  }

  // Optional unicast peers
  bool addPeer(const uint8_t *mac,
               uint8_t channel = 1,
               bool encrypt    = false,
               const uint8_t *lmk16 = nullptr) {
    esp_now_peer_info_t peer{};
    memset(&peer, 0, sizeof(peer));

    memcpy(peer.peer_addr, mac, 6);
    peer.ifidx   = WIFI_IF_STA;
    peer.channel = channel;
    peer.encrypt = encrypt;
    if (encrypt && lmk16) {
      memcpy(peer.lmk, lmk16, 16);
    }

    esp_err_t err = esp_now_add_peer(&peer);
    if (err == ESP_OK || err == ESP_ERR_ESPNOW_EXIST) {
      return true;
    }
    Serial.printf("[ESPNowBase] addPeer failed: %s (0x%X)\n",
                  esp_err_to_name(err), (unsigned)err);
    return false;
  }

  // Broadcast MotionFrame to all nodes
  bool sendFrame(const cro::MotionFrame &frame) {
    const uint8_t *p = reinterpret_cast<const uint8_t*>(&frame);
    size_t len = sizeof(frame);

    const uint8_t *dst = nullptr;
    if (has_bcast_) {
      dst = bcast_mac_;
    }

    esp_err_t err = esp_now_send(dst, p, len);
    if (err != ESP_OK) {
      Serial.printf("[ESPNowBase] esp_now_send failed: %s (0x%X)\n",
                    esp_err_to_name(err), (unsigned)err);
      return false;
    }
    return true;
  }

  // Unicast to specific MAC address
  bool sendFrameTo(const uint8_t *mac, const cro::MotionFrame &frame) {
    const uint8_t *p = reinterpret_cast<const uint8_t*>(&frame);
    size_t len = sizeof(frame);
    esp_err_t err = esp_now_send(mac, p, len);
    if (err != ESP_OK) {
      Serial.printf("[ESPNowBase] esp_now_send(unicast) failed: %s (0x%X)\n",
                    esp_err_to_name(err), (unsigned)err);
      return false;
    }
    return true;
  }

private:
  uint8_t bcast_mac_[6];
  bool    has_bcast_ = false;
};
