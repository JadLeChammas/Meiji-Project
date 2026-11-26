# Projet Emu

Tennis wrist board (MSP430FR2355 + LIS3DH accelerometer + RN4678 Bluetooth).
The board computes the Trunk, Chest, Wrist and Impact events of each shot and their Δt live,
and sends them with the raw data to the PC.

| Folder | Content |
|---|---|
| `Firmware_CCS/Project_Emu/` | Code Composer Studio project of the board (`main.c`) |
| `Test_MCU_Python/live_monitor.py` | Records a session from the board over Bluetooth |
| `Test_MCU_Python/data conversion/` | `export_raw.py`, `pcb_analysis.py`, `shimmer_analysis.py` |
| `Reference/` | Original Python Δt analysis (V2806) |

Python setup (Python 3.14 and the packages): see [`HOW_TO_USE.txt`](HOW_TO_USE.txt).
