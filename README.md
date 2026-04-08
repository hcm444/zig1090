# zig1090

Standalone ADS-B / Mode S demodulator in Zig with an optional web map (Leaflet + `/aircraft.json`) and dump1090-style TCP feeds. RTL-SDR IQ uses **librtlsdr** with a small ring buffer.

**License:** [MIT](LICENSE).

## Build

### Prerequisites

- **Zig**: 0.15.0 or newer.
- **RTL-SDR** drivers and **librtlsdr** dev package:

  - **Linux (Debian/Ubuntu)**:

    ```bash
    sudo apt-get update
    sudo apt-get install -y librtlsdr-dev libusb-1.0-0-dev
    ```

  - **macOS (Homebrew)**:

    ```bash
    brew update
    brew install librtlsdr
    ```

  - **Windows** (example: [vcpkg](https://github.com/microsoft/vcpkg)):

    ```powershell
    git clone https://github.com/microsoft/vcpkg.git
    .\vcpkg\bootstrap-vcpkg.bat
    .\vcpkg\vcpkg install rtlsdr:x64-windows
    ```

    Point Zig at the installed include/lib paths if needed (e.g. `INCLUDE` / `LIB` on Windows).

### Build

```bash
cd zig1090
zig build
```

## Run

Defaults: center `1090e6` Hz, sample rate `2.0e6` Hz. Optional positionals override center and rate.

### Basic

```bash
zig build run
```

### Receiver position (CPR reference)

```bash
zig build run -- --lat 39.528 --lon -119.815
```

### Web map (example: Reno, NV)

```bash
zig build run -- --http 8080 --lat 39.528 --lon -119.815
```

Open `http://127.0.0.1:8080/`.

### dump1090-style TCP (`--net`)

Default listen ports match [FlightAware dump1090](https://github.com/flightaware/dump1090) (`applyNetDefaults`):

| Port | Role |
|------|------|
| 30002 | Raw: `*<hex>;\n` |
| 30003 | BaseStation (SBS) |
| 30005 | Beast cooked binary |

```bash
zig build run -- --net --lat 39.528 --lon -119.815
```

- **`--net-bind-address <ip>`** — default `0.0.0.0`.
- **`--net-ro-port`**, **`--net-sbs-port`**, **`--net-bo-port`** — overrides (any of these enables `--net`).
- **`--net-heartbeat <seconds>`** — keepalives (default `60`, `0` = off).

Beast cooked skips 2-bit CRC–repaired frames. Raw input (30001), Beast input (30004), and Stratux output are not implemented yet.

## Acknowledgments

- **[ZigRadio](https://github.com/vsergeev/zigradio)** — `src/rtl_iq.zig` follows its RTL-SDR IQ scaling and device setup (upstream: Ivan A. Sergeev).
- **ICAO ADS-B / Mode S** — CPR math in `src/adsb/cpr_decode.zig` follows the published standard.
- **[dump1090](https://github.com/flightaware/dump1090) / readsb** — de-facto Mode S message layout and CRC behavior for interoperability.
- **Maps** — [OpenStreetMap](https://www.openstreetmap.org/copyright) & [CARTO](https://carto.com/attributions) tiles; [Leaflet](https://leafletjs.com/).

## Notes

- ADS-B on 1090 MHz is public signaling; “decoding” here means parsing the ME field.
- Sample rates from ~`1.8e6` Hz work for Manchester timing; `2.0e6`, `2.4e6`, `4.0e6` are common RTL-SDR choices.
