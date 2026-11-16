package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import io.github.manosfps1pt.ftclogger.debug.FeedforwardTool;
import io.github.manosfps1pt.ftclogger.debug.MotorTarget;
import io.github.manosfps1pt.ftclogger.debug.SelectableOpMode;

/** Robot-side tool registry for the FTC Advanced debugger app. */
@TeleOp(name = "FTC Advanced Debugger", group = "FTC Advanced")
public final class Debugger extends SelectableOpMode {
    /** Upper bounds; the run form can request lower limits for the selected motor. */
    public static final double MAX_TEST_POWER = 1.0;
    public static final double MAX_TEST_CURRENT_AMPS = 7.0;

    @Override
    protected void configure(Registry registry) {
        registry.folder("biobuzz", "BIOBUZZ", MechanismControls::register);
        registry.folder("motor-tests", "Motor Tests", motors -> {
            registerHub(motors, MotorTarget.Hub.CONTROL_HUB, "control", "Control Hub");
            registerHub(motors, MotorTarget.Hub.EXPANSION_HUB, "expansion", "Expansion Hub");
        });
    }

    private static void registerHub(FolderBuilder motors, MotorTarget.Hub hub,
                                    String hubId, String hubLabel) {
        for (int port = 0; port < 4; port++) {
            final MotorTarget target = new MotorTarget(hub, port);
            final String id = "motors." + hubId + ".port" + port;
            // The app lists these folders as selectable motor targets. Selecting a
            // different folder/tool stops the previous tool before initializing it.
            // Hubs and NextMotor are resolved only when a run starts, so an absent
            // Expansion Hub does not prevent testing a Control Hub motor.
            motors.folder(id, hubLabel + " - Motor Port " + port, motor ->
                    FeedforwardTool.register(motor, id + ".feedforward", target,
                            MAX_TEST_POWER, MAX_TEST_CURRENT_AMPS, true));
        }
    }
}
