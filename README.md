# reflow-oven-ctl

TK

## Features

- Supports monitoring and control via MQTT
- Supports OTA firmware updates
- Safety features:
    - Shuts off if a thermocouple is disconnected or fails
    - Shuts off if the thermocouples disagree by more than 15ºC
    - Shuts off if the oven's temperature does not increase after the heater has been turned on for 55 seconds
- Predefined temperature profiles for:
    - SnBi solder paste
    - SnPb solder paste
    - Drying silica gel packets
    - Preheating the oven
- Accepts user-programmed temperature profiles at runtime
- Allows for powering supplemental heating elements in the toaster oven for faster response
- Allows for blowing cool air into the oven for faster cooling

## License

TK

## Author

Chris Dzombak ([dzombak.com](https://www.dzombak.com), [github.com/cdzombak](https://github.com/cdzombak))
