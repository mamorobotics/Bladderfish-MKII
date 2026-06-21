# Bladderfish MKII

**Bladderfish MKII** is a float that is purpose-designed for the MateROV competition. It uses **two ESP32's** communicating through the **ESP-NOW** protocol to transmit data. In order to measure depth it uses a BlueRobotics **BAR30-SENSOR-R1-RP**, but any I2C pressure sensor will work. It also has a Python control interface for ease of use.

# Current Features
- ESP-NOW Communication
- Pressure Sensing
- Manual Motor Control
- Automatic Depth Profiles with Neutral Buoyancy Control
- Live Pressure Data
- Python Control UI

# How to Run a Profile

1. **Calibrate neutral buoyancy** (once per session or water condition)
   - Run the float in test water until it hovers level
   - Note the pump time in seconds (e.g., 2 seconds)
   - Enter the time in milliseconds in the GUI (e.g., 2000 ms)
   - Click "Set Neutral Buoyancy"

2. **Set profile parameters**
   - Target depth 1: Where to descend (e.g., 2.5 m)
   - Hold time 1: How long to stay there (e.g., 30 s)
   - Target depth 2: Where to go next (e.g., 0.4 m)
   - Hold time 2: How long to stay (e.g., 30 s)

3. **Adjust optional settings** (if needed)
   - Descent Offset: How much extra depth before reversing pump (default 0.3 m)
   - Depth Tolerance: How close to target depth is acceptable (default 0.15 m)

4. **Launch the profile**
   - Click "Launch Profile 1" or "Launch Profile 2"
   - Float calibrates at surface, then executes the dive automatically
   - Returns to surface when complete for data transmission

The float intelligently pumps water in to descend and reverses the pump before reaching target depth to slow down and arrive smoothly.

# TODO
- [x] Add Neutral Buoyancy Control
- [ ] Add Acceleration data to optimize descent
- [ ] Clean up Python UI and make it more user-friendly

# Notes
This project is currently a work in progress and is **heavily assisted by AI development tools.**
