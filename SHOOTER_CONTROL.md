# Shooter velocity control

The shooter remains on Expansion Hub motor port 2; its hood is on servo port 1.
Main TeleOp uses NextMotor's velocity PID + feedforward mode with a ramped
reference. The regular motor event loop must be polled after shooter.update().

- X toggles the shooter. Initial target: 3,000 RPM.
- D-pad Up/Down changes the target by 250 RPM per press, including while off.
- Targets are limited to 0-6,000 RPM. X OFF preserves the selected target.
- The reference rises/falls at 1,500 RPM/s (about two seconds to 3,000 RPM).
- X OFF immediately commands zero power and lets the flywheel coast.
- Restarting uses the measured coasting speed as the initial reference.
- A stalled loop contributes at most 0.1 seconds to the next ramp step.

Calibration: 28 counts/revolution, kS=0.141200930894 and
kV=0.00146941352322 power/(rad/s), using radians/second internally.
These are the supplied normalized-power gains at 12 V. The controller scales
kS, kV and kP by 12 / measured battery voltage (refreshed at 20 Hz). Invalid
voltage readings disable motor output until a valid reading returns. RPM is
the driver's display/control unit only.

kP=0.01 power/(rad/s) is an initial feedback gain; it was not measured by
the kS/kV benchmark. kI, kD and kA are zero. Tune kP on the robot while
comparing target, ramped command, measured RPM and power; lower it if speed
oscillates. A reference ramp limits the requested acceleration; actual motor
acceleration still depends on load and controller tuning. kS supplies an
initial friction compensation when the reference becomes positive.

Telemetry and structured logger channels expose requested, ramped and
measured RPM, speed error, and at-speed state. At-speed requires a finished
ramp and measured velocity within 100 RPM of the selected target. It is
informational and does not automatically actuate the feeder or gate.

Main TeleOp requests 100 Hz snapshots and shooter RPM/status channels. The
actual cadence is limited by hardware reads and the OpMode loop; check
opmode.loopTimeMs in the recording. Driver Station display updates run at
10 Hz. Logger transport/status, queue/drops and voltage validity are displayed.
The queue holds 1024 frames, and STOP allows up to 0.75 seconds for logger drain.

Increase RAMP_RPM_PER_SECOND for a faster velocity-reference ramp: 1500 means
about 2 seconds from 0 to 3000 RPM; 3000 means about 1 second. The ramp does
not limit feedback power after a shot at a constant target. Actual acceleration
depends on available voltage, load and output saturation.
