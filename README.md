[README.md](https://github.com/user-attachments/files/26697365/README.md)
# OTTER

**Haptic MIDI controller** that pairs analog-feel **encoder** control with DAW integration: **Live** and **Studio** modes, **haptic** feedback, **layers** and **pages**, and a **Web Editor** (Chrome) for full configuration.

Part of the **Ash Sound Works** custom tools for live and studio — by sound artist and producer **Ash (Ahn Sunghoon)** · [GitHub](https://github.com/nunosmash) / 
[Manual](http://ashsoundworks.com/otter-manual.html)

[![OTTER](https://raw.githubusercontent.com/nunosmash/OTTER/refs/heads/main/OTTER_img.jpg)](https://youtu.be/qmccIaClYyg?si=hlSnW6YCaZTJ9VbL)

---

## Why OTTER?

Typical MIDI controllers struggle with the same tension: **analog pots** feel great, but mapping them to a DAW can cause **parameter jumps** encoders don’t always fix—especially when acceleration makes muscle-memory control unreliable on stage.

OTTER uses **encoders** with a **Live Mode** that maps rotation to a **fixed angular range** (like a pot), independent of spin speed, plus **haptic** cues at center (64) and at min/max (0 / 127). **Studio Mode** keeps fine resolution for production work. Bidirectional sync with the DAW and the Web Editor keeps names, colors, and lock logic aligned across sessions.

---

## Features

- **4 encoders** with push / rotate, **4 pages** — up to **32 parameter roles** (4×4×2 layers), plus multiple **global** and **button** mappings  
- **Live Mode** — rotation angle maps 1:1 with smoothing (stage-friendly, eyes-off control)  
- **Studio Mode** — speed-based acceleration for fast sweeps and fine edits  
- **Safety Lock** — auto-locks after page changes or idle; ~4 detents rotation to unlock  
- **Haptic motor** + **LED** feedback (center snap, boundary alerts); intensity: OFF / LOW / MID / HIGH  
- **USB MIDI** (class-compliant)  
- **Internal flash** + Web Editor sync for presets  
- **Web Editor** — latest **Chrome** ; connect via WebUSB  

---

## Specifications (summary)

| Item | Details |
|------|---------|
| Model | OTTER |
| Controls | Global / Page encoder, Encoders 1–4 (push + rotate) |
| Pages | 4 (4 encoders × 2 layers per page → up to 32 parameters) |
| I/O | USB MIDI (class-compliant) |
| Feedback | Haptic motor, LEDs |
| Storage | Internal flash, synced with Web Editor |
| Web Editor | Chrome (latest recommended) |

---

## Basic hardware behavior

- **Rotate** — increase / decrease value  
- **Short push** — layer toggle or button action (per mapping)  
- **~4 detents rotation** — unlock **Safety Lock**  
- **Global push + bottom encoder click** — device setup (DIM time, LED brightness, vibration strength, mode)  

---

## Web Editor

1. Use **Chrome** (current version).  
2. Open the bundled **OTTER Web Editor** HTML (local file or hosted URL).  
3. Connect OTTER over USB → click **Connect** in the editor and grant device access.  
4. When the status shows **Connected**, edit and sync parameters, names, and colors.  

Editor path in the site package: `otter/OTTER Web Editor.html`

---

## Firmware update (.uf2)

OTTER uses **UF2** drag-and-drop firmware (RP2040 bootloader appears as **RPI-RP2**).

1. Connect OTTER with a **data** USB cable.  
2. Hold **Button 1** and **Button 4** together for **3+ seconds** until the **RPI-RP2** drive appears.  
3. Copy the correct **`.uf2`** file onto the drive; the device reboots when done.  

---

## Repository contents

Depending on branch or release, this repo may include:

- Firmware source  
- Web Editor assets  
- Build and flash instructions  

For full mode behavior, haptics detail, and troubleshooting, see **`otter-manual.html`** in the Ash Sound Works website bundle — add your public URL once deployed.

---

| Category | Component Description | Qty |
| :--- | :--- | :---: |
| **MCU** | Raspberry Pi Pico 2 (RP2350) | 1 |
| **Display** | 2.25" TFT LCD (ST7789, 76x284 Resolution) | 1 |
| **Encoders (Primary)** | Bourns PEC11R-4015F-S0024 | 4 |
| **Encoder (Menu)** | EC11 Rotary Encoder w/ Push Switch (Detent) | 1 |
| **Haptics** | 3V 10mm Mini Vibration Coin Motor | 1 |
| **Connectors** | PJ-320A 3.5mm Stereo Jack | 1 |
| **Capacitors** | 100μF Electrolytic Capacitor | 2 |
| **Enclosure** | 121 x 43 x 2mm Clear Acrylic Top Plate | 1 |
| **Hardware** | Custom Encoder Knobs | 5 |
| **Hardware** | Brass Heat-Set Insert Nuts (M2x2x3.2) | 4 |
| **Hardware** | M2 Phillips Head Screws (M2x4) | 4 |

---

## Documentation

- Full manual: `otter-manual.html`  
- Web Editor: `otter/OTTER Web Editor.html`  
- Other projects: [github.com/nunosmash](https://github.com/nunosmash)

---

## Disclaimer

- Use a **data-capable** USB cable for editor and firmware updates.  
- DIY builds: follow safe practice for power and assembly.

---

## License

See the `LICENSE` file in the repository root.
