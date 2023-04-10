# FrankenUSBT
The Frankenstein USB HID to BLE HID for ESP32-S3. This projects takes ESP-IDF example on USB Host and BLE HID and hacks together a basic USB to BLE converter for HID class devices. Most of the code is a verbatim copy from the ESP-IDF examples with very minor changes.

<b> No warrenty or guarrenty of any kind, explict or implicit. The software may work, may not work, may destroy your HID device and/or may destroy your development board and/or destroy your computer. </b>

## Current Status
* ONE Keyboard USB to Bluetooth
  * Boot Protocol only
    * Only 6 Key Rollover
    * Some keys like Win/CMD and Fn may not work
  * No support for Host to Device Reports
    * No Num-Lock, Caps-Lock, Scroll Lock indicator LEDSs. However functionality works itself fine.
* Tested on BPI-Leaf-S3 but likely to work on any ESP32-S3 board.
  * ESP32-S3 dev boards do not have powered USB ports and require power from external source.
    * Use a Y-cable like this one : https://www.amazon.ca/Startech-Com-Cable-External-Drive-Usb-USB2HABMY3/dp/B003HHK576 and a USB-C to USB-A converter like this one https://www.amazon.ca/Adapter-JIEconn-Thunderbolt-Compatible-MacBook/dp/B0BKRQZHST .
    * Provide power to board by a Y cable (like above) OR independently via ESP32-S3 pins OR via a second USB port for serial IO (present on some boards).     
