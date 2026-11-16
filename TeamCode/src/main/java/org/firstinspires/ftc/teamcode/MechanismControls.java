package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import dev.nextftc.hardware.actuators.NextMotor;
import io.github.manosfps1pt.ftclogger.debug.SelectableOpMode.Command;
import io.github.manosfps1pt.ftclogger.debug.SelectableOpMode.FolderBuilder;
import io.github.manosfps1pt.ftclogger.debug.SelectableOpMode.Parameter;
import io.github.manosfps1pt.ftclogger.debug.SelectableOpMode.ParameterResult;
import io.github.manosfps1pt.ftclogger.debug.SelectableOpMode.Tool;
import io.github.manosfps1pt.ftclogger.debug.SelectableOpMode.ToolContext;
import io.github.manosfps1pt.ftclogger.protocol.DebugCommandArgumentValue;
import io.github.manosfps1pt.ftclogger.protocol.DebugCommandRequest;
import io.github.manosfps1pt.ftclogger.protocol.DebugCommandResponse;
import io.github.manosfps1pt.ftclogger.protocol.DebugCommandResult;
import io.github.manosfps1pt.ftclogger.protocol.DebugRiskClass;
import org.firstinspires.ftc.teamcode.subsystems.CRWheel;
import org.firstinspires.ftc.teamcode.subsystems.Fos;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Arrays;

/** Operates the same mechanisms as TeleOp under the debugger's control lease. */
final class MechanismControls implements Tool {
    private static final String HOOD_POSITION = "hood.position";
    private static final String SHOOTER_RPM = "shooter.rpm";
    private static final String INTAKE_POWER = "intake.power";
    private static final String GATE_OPEN = "gate.open";
    private static final String GATE_CLOSE = "gate.close";
    private static final String STATUS = "mechanisms.status";

    private ToolContext context;
    private Shooter shooter;
    private Intake intake;
    private CRWheel wheel;
    private Gate gate;
    private Fos indicator;
    private Double hoodPosition;
    private boolean shooterOutput;
    private boolean intakeOutput;

    static void register(FolderBuilder folder) {
        folder.tool("biobuzz.mechanisms", "Mechanism Controls",
                "Set the hood, run the shooter or intake, and test the gate and RGB indicator.",
                DebugRiskClass.DEBUG_MECHANISM, 1.0, true, "", MechanismControls::new,
                Arrays.asList(
                        new Parameter(HOOD_POSITION, "Hood position", "Normalized servo position", 0.5, 0, 1, .01, "", true, false),
                        new Parameter(SHOOTER_RPM, "Shooter target", "Set 0 RPM to stop", 0, 0, Shooter.MAX_TARGET_RPM, 1, "RPM", true, false),
                        new Parameter(INTAKE_POWER, "Intake power", "Positive intakes, negative outtakes, and 0 stops", 0, -1, 1, .01, "", true, false)),
                Arrays.asList(
                        new Command(GATE_OPEN, "Open gate", "Open the gate and turn the indicator red", false),
                        new Command(GATE_CLOSE, "Close gate", "Close the gate and turn the indicator green", false),
                        new Command("stop", "Stop motors", "Stop the shooter, intake, and wheel", false),
                        new Command(STATUS, "Read mechanism status", "Read the current commands and shooter velocity", false)));
    }

    @Override public void init(ToolContext context) { this.context = context; }
    @Override public void start() { }
    @Override public long maxActiveDurationMs() { return 30_000; }

    @Override
    public ParameterResult setParameter(String id, double value, int ttlMs) {
        if (!context.authorized.getAsBoolean()) {
            return ParameterResult.rejected("NOT_ARMED", "Arm the tool to change mechanism output");
        }
        switch (id) {
            case HOOD_POSITION:
                shooter().setHoodPosition(value);
                hoodPosition = value;
                break;
            case SHOOTER_RPM:
                shooter().setTargetRpm(value);
                if (value > 0) {
                    shooterOutput = true;
                    shooter.start();
                } else {
                    stopShooter();
                }
                break;
            case INTAKE_POWER:
                setIntakePower(value);
                break;
            default:
                return ParameterResult.rejected("UNKNOWN_PARAMETER", "Unknown mechanism parameter");
        }
        return ParameterResult.accepted(value);
    }

    @Override
    public DebugCommandResponse command(DebugCommandRequest request) {
        DebugCommandResponse.Builder response = DebugCommandResponse.newBuilder()
                .setRequestId(request.getRequestId()).setCommandId(request.getCommandId())
                .setHandledAtRobotTimeNs(SystemClock.elapsedRealtimeNanos());
        switch (request.getCommandId()) {
            case GATE_OPEN:
            case GATE_CLOSE:
                if (!context.authorized.getAsBoolean()) {
                    return response.setResult(DebugCommandResult.DEBUG_COMMAND_REJECTED)
                            .setMessage("Arm the tool to move the gate").build();
                }
                if (gate == null) gate = new Gate();
                if (indicator == null) indicator = new Fos();
                if (GATE_OPEN.equals(request.getCommandId())) gate.activate();
                else gate.deactivate();
                indicator.showGateState(gate.getState());
                break;
            case STATUS:
                break;
            default:
                return null;
        }
        if (gate != null && indicator != null) {
            response.addOutputs(DebugCommandArgumentValue.newBuilder().setId("gate.state")
                    .setStringValue(gate.getState().name()));
            response.addOutputs(DebugCommandArgumentValue.newBuilder().setId("indicator.position")
                    .setFloat64Value(indicator.getIndicator().getPosition()));
        }
        if (hoodPosition != null) response.addOutputs(number(HOOD_POSITION, hoodPosition));
        if (shooter != null) {
            response.addOutputs(number(SHOOTER_RPM, shooter.isRunning() ? shooter.getTargetRpm() : 0));
            double measured = shooter.getMeasuredRpm();
            if (Double.isFinite(measured)) response.addOutputs(number("shooter.measuredRpm", measured));
        }
        response.addOutputs(number(INTAKE_POWER, intake == null ? 0 : intake.getCurrentPower()));
        return response.setResult(DebugCommandResult.DEBUG_COMMAND_COMPLETED).build();
    }

    private static DebugCommandArgumentValue number(String id, double value) {
        return DebugCommandArgumentValue.newBuilder().setId(id).setFloat64Value(value).build();
    }

    private Shooter shooter() {
        if (shooter == null) shooter = Shooter.forDebugger();
        return shooter;
    }

    private void setIntakePower(double power) {
        if (power == 0) {
            stopIntake();
            return;
        }
        // Ensure even a small nonzero command is followed by hardware writes of zero.
        if (wheel == null) wheel = new CRWheel(0.0);
        if (intake == null) intake = new Intake(0.0);
        intakeOutput = true;
        if (power > 0) intake.intake(power);
        else intake.outtake(-power);
        wheel.setPower(-power);
        intake.hardware().update();
    }

    @Override
    public void loop() {
        if (!context.authorized.getAsBoolean()) {
            stopOutput();
        } else if (shooter != null && shooter.isRunning()) {
            shooter.update();
            shooter.hardware().update();
        }
    }

    private void stopShooter() {
        if (!shooterOutput) return;
        shooter.stop();
        shooter.hardware().update();
        shooterOutput = false;
    }

    private void stopIntake() {
        if (!intakeOutput) return;
        intake.stop();
        try {
            intake.hardware().update();
        } finally {
            wheel.stop();
        }
        intakeOutput = false;
    }

    @Override public void stopOutput() {
        try {
            stopShooter();
        } finally {
            stopIntake();
        }
    }

    @Override public void stop() {
        try {
            stopOutput();
        } finally {
            NextMotor.Companion.getMotorEventLoop().clear();
        }
    }

    @Override public double requestedOutput() { return intake == null ? 0 : intake.getCurrentPower(); }
    @Override public double appliedOutput() { return intake == null ? 0 : intake.hardware().getThrottle(); }
    @Override public long encoderPosition() { return 0; }
    @Override public boolean outputAllowed() {
        return intakeOutput || shooterOutput;
    }
    @Override public boolean watchdogHealthy() { return true; }
}
