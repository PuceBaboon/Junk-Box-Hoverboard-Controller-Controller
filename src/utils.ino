/*
 *   $Id: utils.ino,v 1.3 2021/08/18 23:06:41 gaijin Exp $
 *
 *  Functional utilities common to both HB-controller and
 *  Remote Control unit.
 *
 */


/*
 * Display numeric MAC address in standard format.
 */
void printMacAddress(uint8_t * macaddr) {
    for (int i = 0; i < 6; i++) {
        Serial.print(macaddr[i], HEX);
        if (i < 5)
            Serial.print(':');
    }
    Serial.println();
}


/*
 * The magic incantation to display our own channel number.
 */
void displayWiFiChan() {
    uint8_t wc_one = 254;
    uint8_t *p_wc_one = &wc_one;
    wifi_second_chan_t wc_two = (wifi_second_chan_t) 0;
    wifi_second_chan_t *p_wc_two = &wc_two;
    Serial.print("Channel set to: ");
    ESP_ERROR_CHECK(esp_wifi_get_channel(p_wc_one, p_wc_two));
    if (wc_one == 254) {
        Serial.println("!! CHANNEL NOT SET !! Check ESPNOW_DEFCHAN in include/espnow_common.h");
    } else {
        Serial.println(wc_one, DEC);
    }
}
