^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ds_dbw_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.1 (2026-07-29)
------------------
* Add steering_wheel_angle_error and degraded_angle_sensor signals
* Contributors: Kevin Hallenbeck

2.4.0 (2026-05-28)
------------------
* ROS Lyrical: CMake 3.22 minimum, replace ament_target_dependencies() with target_link_libraries()
* Add signals to RemoteReport
* Add temperature to EcuInfo message
* Add gear reject enumeration for startup
* Add headlights to misc command
* Add SteerTorqueOut message with steering angle controller debug info
* Add option to specify steering torque passthrough percent in command message
* Add signals to PropulsionInfo message
* Contributors: Kevin Hallenbeck

2.3.12 (2026-04-06)
-------------------
* Add air brake message
* Add gear reject shift out-of-park
* Add BrakeDual message for individual channel pressures
* Add one-pedal command and status to DriveMode Cmd/Report
* Add blackout light status
* Add brake actuator temperature
* Contributors: Gabriel Oetjens, Kevin Hallenbeck, Ryan Kaddis

2.3.11 (2026-02-19)
-------------------
* Add internal steering offset enumeration
* Add gear manual number control
* Add door lock statuses to MiscReport
* Add door/trunk open/close control to MiscCmd
* Add support for larger positive ACC brake acceleration commands
* Add 2nd and 3rd row seat-belt statuses
* Add BrakeReport BrakeAvailablePartialInf signal
* Add gear ready to calibrate status
* Contributors: Gabriel Oetjens, Kevin Hallenbeck

2.3.10 (2025-12-18)
-------------------
* Add diagnostic for external brake mismatch between redundant inputs
* Contributors: Kevin Hallenbeck

2.3.9 (2025-10-24)
------------------
* Add gear command return-to-park flag
* Contributors: Gabe

2.3.8 (2025-09-17)
------------------
* Add SystemSyncMode Partial/PartialWithBtn
* Contributors: Kevin Hallenbeck

2.3.7 (2025-08-21)
------------------
* Add hardware disable status
* Contributors: Kevin Hallenbeck

2.3.6 (2025-08-18)
------------------
* Add ThrottleOffset message
* Add firmware info to EcuInfo message
* Contributors: Kevin Hallenbeck

2.3.5 (2025-06-27)
------------------
* Add monitor steer rate debug message
* Add SteerInfo message
* Contributors: Kevin Hallenbeck

2.3.4 (2025-05-06)
------------------
* Add engine/transmission/motor temperatures and over-temperature warnings to MiscReport
* Move GearNum signal from ThrtlInfo to PropulsionInfo and split into GearNumTarget and GearNumActual
* Add steering command type SetOffset to set steering wheel angle offset directly
* Add propulsion torque limiting and limp home statuses (Polaris Ranger XD only)
* Add engine throttle valve position (Polaris Ranger XD only)
* Add brake hold diagnostics
* Contributors: Kevin Hallenbeck

2.3.3 (2025-03-28)
------------------
* Add remote control report (mode/state/timeout)
* Add steering stall limit diagnostic
* Add actuator temperature and warm/hot diagnostics
* Add GPIO/SSR status and control
* Contributors: Kevin Hallenbeck

2.3.2 (2025-02-18)
------------------
* Add drive mode command and report messages
* Add AEB precharge control to brake command and joystick demo
* Add date/time to EcuInfo message
* Add PropulsionInfo message
* Add RBA, CTA, and auto brake hold statuses to DriverAssist message
* Add signals to apply brakes for gear calibration
* Add gear stuck in neutral diagnostics
* Contributors: Gabriel Oetjens, Kevin Hallenbeck

2.3.1 (2024-12-18)
------------------

2.3.0 (2024-11-12)
------------------
* Add traffic sign info message
* Remove deprecated turn signal from MiscReport and MiscCmd
* Add warning for missing brake pedal feedback
* Add brake/throttle yield request flags
* Contributors: Kevin Hallenbeck

2.2.3 (2024-09-30)
------------------
* Add eye tracker message (Mach-E only)
* Add gear manual and gear sport enumerations
* Contributors: Gabriel Oetjens, Kevin Hallenbeck

2.2.2 (2024-09-09)
------------------
* Add warning for brake actuator warm
* Contributors: Kevin Hallenbeck

2.2.1 (2024-08-12)
------------------

2.2.0 (2024-07-17)
------------------
* Add steering offset message
* Add several new messages and signals
  New messages:
  - Battery
  - Low voltage battery state-of-charge/voltage/current/temperature
  - Vehicle ignition
  - BatteryTraction
  - High voltage battery state-of-charge/voltage/temperature
  - DriverAssist
  - ADAS deceleration value
  - FCW/AEB/ACC/BLIS/CTA statuses
  - FuelLevel
  - Fuel level
  - Odometer
  - GPS
  New signals in existing messages:
  - MiscReport
  - Wiper
  - Headlights (high and low beams)
  - Ambient light
  - Outside air temperature
  - ThrottleInfo
  - Drive mode
  - Transmission gear number
* Separate turn signal messages with diagnostics
  Keep functionality in misc cmd/report for a while to ease the transition
* Contributors: Kevin Hallenbeck

2.1.16 (2024-06-17)
-------------------

2.1.15 (2024-06-07)
-------------------
* Brake on comms loss and system lockout
* Contributors: Kevin Hallenbeck

2.1.14 (2024-05-29)
-------------------
* Add gear command option for calibration
* Add gear fault_actuator_config diagnostic
* Contributors: Kevin Hallenbeck

2.1.13 (2024-05-13)
-------------------
* Add gear reject enumeration for excessive vehicle speed
* Add actuator pedal position quality diagnostics
* Add logging status to EcuInfo message
* Contributors: Kevin Hallenbeck

2.1.12 (2024-04-01)
-------------------
* Add support for DBW Monitor module
* Contributors: Kevin Hallenbeck

2.1.11 (2024-03-05)
-------------------
* Add control performance fault
* Add support for system lockout
* Contributors: Kevin Hallenbeck

2.1.10 (2024-02-27)
-------------------

2.1.9 (2024-02-23)
------------------

2.1.8 (2024-02-20)
------------------

2.1.7 (2024-02-12)
------------------
* Add support for system sync option
* Steering wheel angle as degrees instead of radians
* Add brake vacuum pressure
* Contributors: Kevin Hallenbeck

2.1.6 (2024-01-16)
------------------
* Command limits vs vehicle speed
* Report calculated steer value/rate limits
* Report brake/throttle value limits
* Add warning on invalid steer/brake/throttle limit parameters
* Contributors: Kevin Hallenbeck

2.1.5 (2024-01-03)
------------------

2.1.4 (2023-12-13)
------------------
* Single package for all platforms with new DBW2 CAN API
* Contributors: Kevin Hallenbeck
