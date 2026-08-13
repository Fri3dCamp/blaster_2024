## Fri3d Camp 2024 Blaster (Flamingo) firmware

This firmware runs on the [LANA-TNY](https://phyx.be/LANA_TNY) module that powers the Fri3d camp 2024 blaster (flamingo). This module is powered by the [WCH CH32V203 microcontroller](https://www.wch-ic.com/products/CH32V203.html). It handles the IR communication, badge communication though the badgelink, team selection and LED effects of the blaster.

## Building

Use [PlatformIO](https://platformio.org) to build this project. You should install the [ch32v platform package](https://github.com/Community-PIO-CH32V/platform-ch32v) as well:

```
pio pkg install -g -p https://github.com/Community-PIO-CH32V/platform-ch32v.git
```

There are 2 build environments available:
 * `debug`: builds with debug output enabled (`DEBUG=2`) and a placeholder version number.
 * `release`: the environment used to build official releases, with the version number set via build flags from a release tag.

Select an environment with `-e`, e.g.:

```
pio run -e debug
```

When flashing a `debug` build, debug messages (using `PRINT()` macro) are printed over UART (baudrate `115200 8N1`) on the blaster's P5 connector TX pin.

## Flashing

To flash your 2024 blaster, unplug the USB cable, press and hold the boot button of the [LANA-TNY](https://phyx.be/LANA_TNY) module while plugging in the USB cable again. Then upload using the command:

```
pio run -e debug -t upload
```

It will use [wchisp](https://github.com/Community-PIO-CH32V/tool-wchisp) to flash the binary to the CH32V203 chip of the [LANA-TNY](https://phyx.be/LANA_TNY) module.

Alternatively, you can flash the latest release of the firmware straight from your browser using the [Fri3d web flasher](https://fri3dcamp.github.io/fri3d-web-flasher/), without needing to install PlatformIO.

## Releases

Pushing a `v*` tag triggers the [release workflow](../../.github/workflows/release.yml), which builds the `release` environment, publishes the firmware binary as a GitHub release and uploads it to [BadgeHub](https://badgehub.eu).
