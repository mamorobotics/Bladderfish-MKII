# Bladderfish Float Calibration System

Automatic motor speed calibration with GUI interface.

## Files

**Two Separate Firmware:**
- `Float_Speed_Test_Shallow.ino` - Calibration test (finds optimal speeds)
- `/Float/Float.ino` - Main float controller (uses calibrated values)

**Calibration GUI:**
- `python_controller.py` - GUI for running calibration tests

**Config & Dependencies:**
- `calibration_config.json` - Stores learned PWM values
- `requirements.txt` - Python dependencies

## Quick Start

### 1. Run Calibration (One-Time)
```bash
python python_controller.py
```
- Click "Connect" (select COM port)
- Click "Start Calibration Test"
- Float tests 7 motor speeds automatically
- GUI displays results with graphs
- Values saved to `calibration_config.json`

### 2. Update Main Float Code
Edit `/Float/Float.ino` - update calibration variables at top:
```cpp
// ════════════════════════════════════════════════════════════════════════════
// CALIBRATION VARIABLES - EDIT THESE AFTER RUNNING CALIBRATION TEST
// ════════════════════════════════════════════════════════════════════════════

int DESCENT_PWM = 180;        // From calibration test result
int ASCENT_PWM = 185;         // From calibration test result
int FINE_CONTROL_PWM = 75;
int RAPID_DESCENT_PWM = 240;
```

### 3. Upload & Deploy
- Upload updated `Float.ino` to float
- All missions automatically use optimized PWM values

## Calibration GUI Features

- **Real-time monitoring**: Live data points and depth graph
- **Automatic analysis**: Finds optimal speeds targeting 0.5 m/s
- **Multiple export options**: CSV (raw data), TXT (report), graphs
- **Results display**: Shows descent/ascent rates for each speed

## Expected Results

```
DESCENT RATES:
PWM  80:  0.2145 m/s
PWM 100:  0.2845 m/s
PWM 120:  0.3521 m/s
PWM 140:  0.4123 m/s
PWM 160:  0.4856 m/s
PWM 180:  0.5234 m/s    ← OPTIMAL (closest to 0.5 m/s target)
PWM 200:  0.5891 m/s

UPDATE Float.ino:
int DESCENT_PWM = 180;
int ASCENT_PWM = 185;
```

## How It Works

1. **Calibration Test** (Float_Speed_Test_Shallow.ino):
   - Tests 7 motor speeds (80-200 PWM)
   - Measures descent/ascent rates
   - Collects ~5000 sensor data points
   - Returns to surface with all data

2. **GUI Analysis** (python_controller.py):
   - Receives data from float
   - Calculates rates for each speed
   - Finds optimal speeds (targets 0.5 m/s)
   - Auto-saves to calibration_config.json

3. **Main Float Usage** (/Float/Float.ino):
   - Loads DESCENT_PWM and ASCENT_PWM at top
   - All missions use these calibrated speeds
   - Functions: `smartDescend()` and `smartAscend()` auto-use values

## Workflow Example

```
1. Run calibration test GUI
   python python_controller.py
   
2. Results shown:
   Optimal descent: PWM 180
   Optimal ascent: PWM 185
   
3. Update Float.ino top variables:
   int DESCENT_PWM = 180;
   int ASCENT_PWM = 185;
   
4. Upload Float.ino
   
5. All profiles now use optimal speeds automatically
```

## Installation

```bash
pip install -r requirements.txt
```

Requires: `pyserial`, `matplotlib` (optional, for graphs)

---

**Version**: 1.0 | 2026-06-21  
**Calibration Target**: 0.5 m/s descent/ascent  
**Test Depth**: 2m with ±10cm accuracy
