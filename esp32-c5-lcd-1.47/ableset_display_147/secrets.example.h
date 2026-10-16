// Copy this file to "secrets.h" in the same folder and fill in your networks.
// secrets.h is listed in .gitignore, so your passwords never end up on GitHub.
//
// The display tries these networks in order (2.4 GHz or 5 GHz both work on
// the ESP32-C5) and moves on to the next one if AbleSet can't be found.

static const WifiNetwork WIFI_NETWORKS[] = {
  { "BandWiFi",  "your-password" },
  { "HomeWiFi",  "your-password" },
};
