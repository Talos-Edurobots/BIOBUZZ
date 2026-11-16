package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import io.github.manosfps1pt.ftclogger.StructuredRobotDataClient;
import io.github.manosfps1pt.ftclogger.LoggerConfig;
import dev.nextftc.hardware.actuators.NextMotor;
import io.github.manosfps1pt.ftclogger.NextBindings;
import io.github.manosfps1pt.ftclogger.SignalChannel;
import io.github.manosfps1pt.ftclogger.protocol.ChannelRole;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends OpMode {
    private static final int CAPTURE_HZ = 100;
    private long lastDisplayNanos;
    private Robot robot;
    private StructuredRobotDataClient client;
    private final ElapsedTime runtime = new ElapsedTime();
    double dt = 0, lastTime = 0;

    @Override
    public void init() {
        // NextMotor binds each new instance to a shared loop. Drop stale bindings first.
        NextMotor.Companion.getMotorEventLoop().clear();
        try {
        robot = new Robot();
        LoggerConfig defaults = LoggerConfig.defaults();
        client = new StructuredRobotDataClient("Main TeleOp")
                .configure(new LoggerConfig(defaults.robotId, defaults.transport, CAPTURE_HZ, 750,
                        1024, defaults.retainedRecordings, defaults.recordingBytes))
                .addRuntime(runtime).addGamepads(gamepad1, gamepad2);
        java.util.Iterator<com.qualcomm.robotcore.hardware.VoltageSensor> voltages = hardwareMap.voltageSensor.iterator();
        if (voltages.hasNext()) client.addVoltageSensor(voltages.next());
        NextBindings.addMotor(client,"intake","Intake Motor",robot.intake.hardware(),robot.intake::getCurrentPower);
        NextBindings.addMotor(client,"shooter","Shooter Motor",robot.shooter.hardware(),robot.shooter::getAppliedPower);
        client.addRgbIndicator("fos", "Gate RGB Indicator", robot.fos.getIndicator()::getPosition);
        client.addChannel(SignalChannel.number("shooter.targetRpm", "Shooter target RPM", "shooter", "velocity", "RPM", ChannelRole.TARGET, CAPTURE_HZ, robot.shooter::getTargetRpm));
        client.addChannel(SignalChannel.number("shooter.commandedRpm", "Shooter ramped RPM", "shooter", "velocity", "RPM", ChannelRole.COMMAND, CAPTURE_HZ, robot.shooter::getCommandedRpm));
        client.addChannel(SignalChannel.number("shooter.measuredRpm", "Shooter measured RPM", "shooter", "velocity", "RPM", ChannelRole.MEASURED, CAPTURE_HZ, robot.shooter::getMeasuredRpm));
        client.addChannel(SignalChannel.number("shooter.speedErrorRpm", "Shooter speed error", "shooter", "velocity", "RPM", ChannelRole.ERROR, CAPTURE_HZ, robot.shooter::getSpeedErrorRpm));
        client.addChannel(SignalChannel.flag("shooter.atSpeed", "Shooter at speed", "shooter", ChannelRole.STATUS, CAPTURE_HZ, robot.shooter::isAtSpeed));
        NextBindings.addMotor(client,"drivetrain.frontLeft","Front Left Motor",robot.drivetrain.frontLeft);
        NextBindings.addMotor(client,"drivetrain.frontRight","Front Right Motor",robot.drivetrain.frontRight);
        NextBindings.addMotor(client,"drivetrain.backLeft","Back Left Motor",robot.drivetrain.backLeft);
        NextBindings.addMotor(client,"drivetrain.backRight","Back Right Motor",robot.drivetrain.backRight);
        client.addChannel(SignalChannel.number("shooter.supplyVoltage", "Shooter compensation voltage", "shooter", "voltage", "V", ChannelRole.MEASURED, 20, robot.shooter::getSupplyVoltage));
        client.addChannel(SignalChannel.flag("shooter.voltageValid", "Shooter voltage valid", "shooter", ChannelRole.STATUS, CAPTURE_HZ, robot.shooter::isVoltageValid));

        client.start();
        } catch (RuntimeException error) {
            try { stop(); } catch (RuntimeException cleanup) { error.addSuppressed(cleanup); }
            throw error;
        }
    }

    @Override
    public void start() {
        runtime.reset();
        client.markStarted();
        robot.turret.activate();
    }

    @Override
    public void loop() {
        dt = runtime.seconds() - lastTime;
        lastTime = runtime.seconds();

        robot.drivetrain.drive(gamepad1);

        if (gamepad1.aWasPressed()) {
            robot.intake.intake(1.0);
            robot.crWheel.setPower(-1.0);
        }

        if (gamepad1.yWasPressed()) {
            robot.intake.outtake(1.0);
            robot.crWheel.setPower(1.0);
        }
        if (gamepad1.bWasPressed()) {
            robot.intake.stop();
            robot.crWheel.stop();
        }
        if (gamepad1.rightTriggerWasPressed()) robot.gate.toggle();
        robot.fos.showGateState(robot.gate.getState());
//        robot.fos.changeBrightness(gamepad1.right_stick_y * dt);
        if (gamepad1.xWasPressed()) {
            if (robot.shooter.isRunning()) {
                robot.shooter.stop();
            } else {
                robot.shooter.start();
            }
        }
        if (gamepad1.leftTriggerWasPressed()) {
            robot.flower.toggle();
        }
        if (gamepad1.dpadUpWasPressed()) robot.shooter.adjustTargetRpm(Shooter.TARGET_STEP_RPM);
        if (gamepad1.dpadDownWasPressed()) robot.shooter.adjustTargetRpm(-Shooter.TARGET_STEP_RPM);


        if (gamepad1.dpadLeftWasPressed()) robot.turret.adjustPosition(-0.1);
        if (gamepad1.dpadRightWasPressed()) robot.turret.adjustPosition(0.1);
        if (gamepad1.startWasPressed()) robot.turret.activate();
        if (gamepad1.backWasPressed()) robot.turret.deactivate();

        if (gamepad1.leftBumperWasPressed()) robot.shooter.changeHoodPosition(-0.1);
        if (gamepad1.rightBumperWasPressed()) robot.shooter.changeHoodPosition(0.1);

        robot.shooter.update();
        // NextMotor applies the requested powers when its motor loop is polled.
        NextMotor.Companion.getMotorEventLoop().poll();

        // Capture before the slower Driver Station display work. Actual rate follows loop cadence.
        client.publishLoop();
        long displayNow = System.nanoTime();
        if (displayNow - lastDisplayNanos < 100_000_000L) return;
        lastDisplayNanos = displayNow;

        telemetry.addLine("Controls: A intake + wheel | B stop intake + wheel | Y outtake + wheel | Right trigger gate | X shooter on/off | D-pad Up/Down shooter +/-250 RPM | D-pad Left/Right turret position | Start/Back turret PWM on/off | Bumpers hood");
        telemetry.addData("Shooter state", robot.shooter.isRunning() ? "RUNNING" : "OFF");
        telemetry.addData("Shooter target", "%.0f RPM", robot.shooter.getTargetRpm());
        telemetry.addData("Shooter ramped command", "%.0f RPM", robot.shooter.getCommandedRpm());
        telemetry.addData("Shooter measured", "%.0f RPM", robot.shooter.getMeasuredRpm());
        telemetry.addData("Shooter speed error", "%.0f RPM", robot.shooter.getSpeedErrorRpm());
        telemetry.addData("Shooter at speed", robot.shooter.isAtSpeed());
        telemetry.addData("Shooter applied power", "%.2f", robot.shooter.getAppliedPower());
        telemetry.addData("Turret commanded position", "%.2f", robot.turret.getTargetPosition());
        telemetry.addData("Turret PWM active", robot.turret.isActive());
        telemetry.addData("Intake power", "%.2f", robot.intake.getCurrentPower());
        telemetry.addData("Gate state", robot.gate.getState());
        telemetry.addData("Intake current", "%.2f A",
                robot.intake.hardware().getCurrent().getMagnitude());
        telemetry.addData("Shooter current", "%.2f A",
                robot.shooter.hardware().getCurrent().getMagnitude());
        telemetry.addData("Shooter compensation voltage", "%.2f V", robot.shooter.getSupplyVoltage());
        telemetry.addData("Shooter voltage valid", robot.shooter.isVoltageValid());
        telemetry.addData("Logger", "%s / %s", client.transportMode(), client.status());
        telemetry.addData("Logger sample interval", "%.1f ms requested", 1000.0 / CAPTURE_HZ);
        telemetry.addData("Logger queue / dropped", "%d / %d", client.getQueuedPacketCount(), client.getDroppedPacketCount());
        telemetry.addData("Logger error", client.getLastError());
        telemetry.update();
    }

    @Override
    public void stop() {
        try {
            if (robot != null) robot.close();
        } finally {
            try {
                NextMotor.Companion.getMotorEventLoop().poll();
            } finally {
                // Flush the stop commands before discarding this run's motor callbacks.
                NextMotor.Companion.getMotorEventLoop().clear();
                if (client != null) client.close();
            }
        }
    }
}
