# Lineform AI

## Structure

- `gui/Gui_3_0/` – Processing sketch (open `Gui_3_0.pde` in Processing). All `.pde` tabs must stay in this folder.
  - `local_config.json` – per-arm GUI settings (`active_motors`, `base_block_first`). Git-ignored; copy `local_config.example.json` to create it.
  - `shape_library.json` – saved shape library, loaded at startup.
  - `logs/` – runtime output: `ai_debug_log.txt` (overwritten each run, not committed) and `ai_fit_history.csv` (fit-quality history across runs).
- `firmware/lineform-ai/` – Arduino sketch (`lineform-ai.ino`). Serial at 115200 baud.
  - `local_config.h` – per-arm settings (reversed motors, `ACTIVE_MOTORS`). Git-ignored; copy `local_config.example.h` to create it.
- `firmware/tools/` – one-time motor setup sketches (not the robot firmware; re-upload `lineform-ai.ino` afterwards):
  - `Set_Motor_ID/` – scan the bus and give ONE motor a new ID.
  - `Shift_Motor_IDs/` – shift every connected motor's ID by ±1. Starts in `DRY_RUN = true` (prints the plan only).
- `update_notes/` – dated notes of changes (one file per day, `YYYY-MM-DD.md`).
- `docs/` – wiring, notes, photos.

## Setup

Put your Anthropic API key in `gui/Gui_3_0/anthropic_key.txt`. This file is git-ignored; never commit it.
