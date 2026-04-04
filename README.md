# zig1090

Standalone ADS-B / Mode S demodulator in Zig with an optional local “net mode” web UI (Leaflet map + `/aircraft.json`).
RTL-SDR IQ is read via **librtlsdr** with a small local ring buffer (see [Acknowledgments](#acknowledgments)).

<img width="1934" height="1603" alt="Screenshot_20260324_220616" src="https://github.com/user-attachments/assets/deda21d2-77e0-4c5c-b0dd-2557e43c5c46" />


## Build

### Prerequisites

- **Zig**: 0.15.0 or newer.
- **RTL-SDR runtime & drivers** as appropriate for your dongle/OS.
- **librtlsdr** development libraries:

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

  - **Windows**:
    - One option is to use [vcpkg](https://github.com/microsoft/vcpkg):

      ```powershell
      git clone https://github.com/microsoft/vcpkg.git
      .\vcpkg\bootstrap-vcpkg.bat
      .\vcpkg\vcpkg install rtlsdr:x64-windows
      ```

    - Ensure the installed `rtlsdr` include/lib paths are visible to Zig (e.g. via `INCLUDE` / `LIB` env vars) if your setup differs from the CI workflow.

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

### With receiver position (CPR reference)

```bash
zig build run -- --lat 39.528 --lon -119.815
```

### Net mode example (Reno, NV)

Starts the local web UI on port `8080` and sets the receiver position to Reno (approx. `39.528, -119.815`):

```bash
zig build run -- --http 8080 --lat 39.528 --lon -119.815
```

Then open `http://127.0.0.1:8080/`.

## Acknowledgments

- **[ZigRadio](https://github.com/vsergeev/zigradio)** — [zigradio.org](https://zigradio.org). Copyright (c) Ivan (Vanya) A. Sergeev; **MIT License**. `src/rtl_iq.zig` follows the device setup and IQ scaling of ZigRadio’s `RtlSdrSource`; zig1090 no longer depends on ZigRadio as a package.

- **ICAO / ADS-B MOPS** — CPR position decoding in `src/adsb/cpr_decode.zig` implements the published compact-position-reporting math (same standard as commercial and hobby decoders).

- **dump1090 / readsb** — Mode S message length (`df & 0x10`), CRC-24 parity, and several ME field layouts match the de-facto behavior of [dump1090](https://github.com/flightaware/dump1090) and similar decoders for interoperability.

## Notes

- ADS-B on 1090 MHz is not encrypted; “decoding” here means parsing the public ME field.
- Sample rates ≥ ~`1.8e6` work (Manchester timing); `2.0e6`, `2.4e6`, `4.0e6` are common RTL-SDR choices.
