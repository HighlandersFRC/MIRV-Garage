# MIRV-Garage
Mirv Garage Control Code

## Introduction
The Mirv Garage is controlled from a Heltec ESP32 V2 Wireless board. The ESP32 is programmed in cpp, and uses the platform.io extensions for vscode to build and deploy code to the esp32. 


## Setting up Development Environment 
For this project, our team used the VScode extension for platform io to program the garage. PlatformIO provides a great resource on how to set this up [here](https://docs.platformio.org/en/stable/integration/ide/vscode.html).


## Building and Deploying
Before building the project, you will need to modify the config.h file and populate it with the correct parameters. All values in the config file must be populated. Below is a description of each variable and what values it can accept.
| Variable      | Type | Example |Description    |  
| :---        |    :----:   |    :----:   |          :---: |
| ssid      | char*       | "MIRV_WIFI"   | The SSID of the wifi network the garage will connect to. Note, the ESP32 only supports 2.4 GHz Wifi networks.            |
| wifiPassword   | char*        | "1234"      | The Wifi password the garage should use to connect to the provided wifi network. |
| apiHost   | String        | "my-api-site.com"      | The URL of the the API in the cloud. |
| apiPort   | int        | 443      | The port the API will respond to. |
| garageID   | String        | "garage_1"      | A short readable name to know which garage this is. This should be a globally unique value, using the same garage name as another garage will cause connection issues. |
| username   | String        | "mirv-garage"      | The username the garage should use when authenticating to keycloak.  |
| apiPassword   | String        | "1234"     | The password the garage should use when connecting to keycloak. |
| roverID   | String        | "rover_1"      | The rover id of the rover that this garage is paired with. |

Once all values are populated, press the checkmark button in the bottom left corner to build the garage code. PlatformIO should automatically install any external libraries needed.

To deploy to the garage connect a USB A to USB micro cable between the host machine and the garage programing port, which is located on the front panel of the garage box. Note, it is not necessary to open the garage electronic casing to access the programming port. Once connected press the arrow icon in the bottom left corner of platform io to deploy code to the garage. The garage programming port can also be used to receive serial data from the ESP32. This is useful for debugging the device or testing if everything is working properly.
