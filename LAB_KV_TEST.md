# Motor kV test from the laptop

Install `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk` on the Robot
Controller. This build supports laptop startup and arming; the previous APK
requires the official DS/gamepad and must be replaced.

1. Connect the laptop to the robot network and start FTC_ADVANCED's
   `run_web_driver_station.bat`. Restart that backend after updating the code,
   then refresh the browser.
2. On the laptop Driver Station page, connect to the Robot Controller IP.
   No separate Driver Station device or gamepad is needed.
3. Open Debugger and click **Start debugger from laptop**. This stops the
   current OpMode and initializes/starts `FTC Advanced Debugger`.
   The debugger enables its own pit TCP session; Data Transport Setup is
   unnecessary and the persistent logging policy for other OpModes is unchanged.
4. Select the debugger's robot session, then the desired Control Hub or
   Expansion Hub motor port (0-3), and select **kS / kV Calculator**.
5. Enter encoder counts per revolution for the characterized shaft, including
   gearing, direction, maximum power, and current cutoff. The form defaults
   to full power (1.0); choose limits appropriate for the setup. The shaft
   must be free to rotate for the entire characterization. Current is capped
   at 7 A by this registration.
6. Click **Acquire debugger control**, then **Arm selected motor**, then run
   the benchmark. Keep the browser focused and connected. Losing control or
   connection stops output; acquire and arm again to start another run.
7. Wait for transfer and the report. Review fit quality and use the exported
   NextMotor gains for the unit of your velocity setpoint. Voltage-based kV
   and normalized-power kV are different quantities.
8. To change motors, finish/abort the current test, let the shaft stop,
   select another hub/port and calculator, then arm that tool before running.
   Use the laptop robot controls' STOP button to stop the whole OpMode.

The default test uses 20 power steps, 3 seconds settling and 1 second sampling
per step, plus breakaway and coast-down. Allow roughly two minutes.
Forward means positive raw port power; it does not inherit a subsystem's
reversed motor setting. A missing Expansion Hub does not prevent testing a
Control Hub motor.

## Building from source

BIOBUZZ pins `ftcloggerVersion` to `1.0.0-rc.1-laptop.1`, published to this
machine's Maven local repository. To reproduce on another machine, use the
sibling ftclogger checkout with the laptop-arm changes and publish all modules:

```powershell
# In ftclogger
./gradlew.bat publishReleasePublicationToMavenLocal :debugger:publishReleasePublicationToMavenLocal :nextftc:publishReleasePublicationToMavenLocal '-Pversion=1.0.0-rc.1-laptop.1'
# In BIOBUZZ
./gradlew.bat :TeamCode:assembleDebug
```
