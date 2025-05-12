# NPG Lite - Firmware

Neuro PlayGround (NPG) Lite firmware repository.

[Support NPG-Lite project by signing-up with your email on CrowdSupply](https://www.crowdsupply.com/upside-down-labs/neuro-playground-lite)

| Example | Code |
| ------- | ---- |
| Default program to show Visual, Audiotory, and Haptic feedback on NPG| [NPG-Default.ino](NPG-Default/NPG-Default.ino) |
| Calculate FFT & Band Power of single channel EEG and print on Serial | [Serial-FFT.ino](Serial-FFT/Serial-FFT.ino) |
| Bluetooh Low Energy (BLE) server to notify client with real-time NPG data | [BLE-Server.ino](BLE-Server/BLE-Server.ino) |
| BLE client to take notification from server and trigger GPIO | [BLE-Client.ino](BLE-Client/BLE-Client.ino) |
| BLE server to nofity client based on EEG band (beta) power triggers | [BLE-BCI-Server-Toggle.ino](BLE-BCI-Server-Toggle/BLE-BCI-Server-Toggle.ino) |
| InfraRed (IR) reciever code to identify LG AC remote button commands | [IR-LG-Receive.ino](IR-LG-Receive/IR-LG-Receive.ino) |
| IR signal send example code to control LG AC to toggle ON/OFF using user button | [IR-LG-Send.ino](IR-LG-Send/IR-LG-Send.ino) |
| Brain Computer Interface (BCI) to toggle LG AC ON/OFF using EEG band (beta) power | [BCI-IR-Send.ino](BCI-IR-Send/BCI-IR-Send.ino) |

