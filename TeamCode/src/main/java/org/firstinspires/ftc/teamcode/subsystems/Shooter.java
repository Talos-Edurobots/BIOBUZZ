package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.units.Units;
import java.util.Objects;
import java.util.function.LongSupplier;
import java.util.function.DoubleSupplier;


public class Shooter {
    public enum ControlMode { VELOCITY, OPEN_LOOP }

    // Select OPEN_LOOP here to use the power ramp in MainTeleOp.
    private static final ControlMode DEFAULT_CONTROL_MODE = ControlMode.VELOCITY;
    // Benchmark gains use normalized power and rad/s at a 12 V reference.
    public static final double ENCODER_COUNTS_PER_REVOLUTION = 28.0;
    public static final double VELOCITY_KS = 0.0516608255;
    public static final double VELOCITY_KV = 0.00184979817118;
    // Initial feedback gain, not a benchmark result. Tune on the robot in power/(rad/s).
    public static final double VELOCITY_KP = 0.01;
    public static final double DEFAULT_TARGET_RPM = 3000.0;
    public static final double MAX_TARGET_RPM = 6000.0;
    public static final double TARGET_STEP_RPM = 250.0;
    public static final double RAMP_RPM_PER_SECOND = 1500.0;
    public static final double RAMP_POWER_PER_SECOND = 0.3;
    public static final double REFERENCE_VOLTAGE = 12.0;
    private static final double MIN_VALID_VOLTAGE = 6.0;
    private static final long VOLTAGE_SAMPLE_INTERVAL_NS = 50_000_000L;
    private static final double SPEED_TOLERANCE_RPM = 100.0;
    private static final double MAX_RAMP_DT_SECONDS = 0.1;
    private static final double RAD_PER_SECOND_PER_RPM = 2.0 * Math.PI / 60.0;
    private static final double RADIANS_PER_COUNT = 2.0 * Math.PI / ENCODER_COUNTS_PER_REVOLUTION;
    private static Shooter instance;
    // Hardware devices controlled by this subsystem.
    private final NextMotor motor;
    private final NextServo hood;
    // Current commanded hood position, initially centered at 0.5.
    private double hoodPosition = 0.5;

    private final LongSupplier clock;
    private final DoubleSupplier voltageSource;
    private long lastVoltageReadNanos = Long.MIN_VALUE;
    private double supplyVoltage = Double.NaN;
    private boolean voltageValid;
    private final ControlMode controlMode;
    private double targetRpm = DEFAULT_TARGET_RPM;
    private double commandedRpm = 0.0;
    private double commandedPower = 0.0;
    private long lastUpdateNanos;
    private boolean running = false;

    public static synchronized Shooter get() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    public static synchronized void setInstance(Shooter shooter) {
        instance = shooter;
    }

    public Shooter() {
        this(DEFAULT_CONTROL_MODE);
    }

    public Shooter(ControlMode controlMode) {
        this(controlMode, controlMode == ControlMode.OPEN_LOOP ? 0.0 : 0.01, 0.01);
    }

    /** Uses exact hardware writes for the debugger's position and stop commands. */
    public static Shooter forDebugger() {
        return new Shooter(DEFAULT_CONTROL_MODE, 0.0, 0.0);
    }

    private Shooter(ControlMode controlMode, double motorCacheTolerance, double hoodCacheTolerance) {
        // Send small open-loop steps all the way to the hub, including the last step to zero.
        this(new NextMotor(RobotController.expansionHub(), 2, Units.getRadians(RADIANS_PER_COUNT),
                        motorCacheTolerance),
                new NextServo(RobotController.expansionHub(), 1, hoodCacheTolerance), System::nanoTime,
                () -> RobotController.getInputVoltage().getBaseUnitMagnitude(), controlMode);
    }

    public Shooter(NextMotor motor, NextServo hood) {
        this(motor, hood, System::nanoTime);
    }

    Shooter(NextMotor motor, NextServo hood, LongSupplier clock) {
        this(motor, hood, clock, () -> RobotController.getInputVoltage().getBaseUnitMagnitude());
    }

    Shooter(NextMotor motor, NextServo hood, LongSupplier clock, DoubleSupplier voltageSource) {
        this(motor, hood, clock, voltageSource, ControlMode.VELOCITY);
    }

    Shooter(NextMotor motor, NextServo hood, LongSupplier clock, DoubleSupplier voltageSource,
            ControlMode controlMode) {
        this.motor = motor;
        this.hood = hood;
        this.clock = clock;
        this.voltageSource = voltageSource;
        this.controlMode = Objects.requireNonNull(controlMode);
        motor.setAnglePerCount(Units.getRadians(RADIANS_PER_COUNT));
        motor.getVelocityConstants().withS(VELOCITY_KS).withV(VELOCITY_KV)
                .withP(VELOCITY_KP).withI(0.0).withD(0.0).withA(0.0);
        motor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);
        motor.setThrottle(0.0);
    }

    /** Starts at measured speed in velocity mode or zero power in open-loop mode. */
    public void start() {
        if (running) return;
        if (controlMode == ControlMode.VELOCITY) commandedRpm = clampRpm(getMeasuredRpm());
        lastUpdateNanos = clock.getAsLong();
        running = true;
        if (controlMode == ControlMode.OPEN_LOOP) {
            commandedPower = 0.0;
            motor.setThrottle(0.0);
        } else {
            applyVelocityCommand();
        }
    }

    /** Advance the active ramp before NextMotor's event loop applies the command. */
    public void update() {
        if (!running) return;
        long now = clock.getAsLong();
        // A delayed loop must not turn into one large setpoint jump on recovery.
        double dt = Math.max(0.0, Math.min(MAX_RAMP_DT_SECONDS, (now - lastUpdateNanos) / 1e9));
        lastUpdateNanos = now;
        if (controlMode == ControlMode.OPEN_LOOP) {
            applyOpenLoopCommand(dt);
        } else {
            commandedRpm = slew(commandedRpm, targetRpm, RAMP_RPM_PER_SECOND * dt);
            applyVelocityCommand();
        }
    }

    private void applyOpenLoopCommand(double dt) {
        refreshSupplyVoltage();
        commandedPower = voltageValid
                ? slew(commandedPower, openLoopTargetPower(), RAMP_POWER_PER_SECOND * dt)
                : 0.0;
        motor.setThrottle(commandedPower);
    }

    private double openLoopTargetPower() {
        if (targetRpm == 0.0) return 0.0;
        double nominalPower = VELOCITY_KS + VELOCITY_KV * targetRpm * RAD_PER_SECOND_PER_RPM;
        return Math.min(1.0, nominalPower * REFERENCE_VOLTAGE / supplyVoltage);
    }

    private void applyVelocityCommand() {
        if (refreshSupplyVoltage() && voltageValid) {
            double scale = REFERENCE_VOLTAGE / supplyVoltage;
            motor.getVelocityConstants().withS(VELOCITY_KS * scale).withV(VELOCITY_KV * scale)
                    .withP(VELOCITY_KP * scale);
        }
        if (!voltageValid || commandedRpm <= 0.0) {
            // Do not apply negative PID braking to a flywheel that should coast at zero.
            motor.setThrottle(0.0);
        } else {
            motor.setVelocitySetpoint(Units.getRadiansPerSecond(commandedRpm * RAD_PER_SECOND_PER_RPM));
        }
    }

    private boolean refreshSupplyVoltage() {
        long now = clock.getAsLong();
        if (lastVoltageReadNanos != Long.MIN_VALUE
                && now - lastVoltageReadNanos < VOLTAGE_SAMPLE_INTERVAL_NS) return false;
        lastVoltageReadNanos = now;
        try { supplyVoltage = voltageSource.getAsDouble(); }
        catch (RuntimeException unavailable) { supplyVoltage = Double.NaN; }
        voltageValid = Double.isFinite(supplyVoltage) && supplyVoltage >= MIN_VALID_VOLTAGE;
        return true;
    }

    public double getSupplyVoltage() { return supplyVoltage; }
    public boolean isVoltageValid() { return voltageValid; }

    public void adjustTargetRpm(double changeRpm) {
        setTargetRpm(targetRpm + changeRpm);
    }

    public void setTargetRpm(double rpm) {
        if (!Double.isFinite(rpm)) throw new IllegalArgumentException("Shooter RPM must be finite");
        targetRpm = clampRpm(rpm);
    }

    public double getTargetRpm() { return targetRpm; }
    /** Velocity reference, or zero in open-loop mode where the command is power. */
    public double getCommandedRpm() { return commandedRpm; }
    public double getMeasuredRpm() {
        return motor.getEncoderVelocity().getBaseUnitMagnitude() / RAD_PER_SECOND_PER_RPM;
    }
    public double getSpeedErrorRpm() { return targetRpm - getMeasuredRpm(); }
    public boolean isAtSpeed() {
        if (!running || !voltageValid || targetRpm <= 0.0) return false;
        boolean rampComplete = controlMode == ControlMode.OPEN_LOOP
                ? Math.abs(commandedPower - openLoopTargetPower()) < 1e-6
                : Math.abs(commandedRpm - targetRpm) < 1e-6;
        return rampComplete && Math.abs(getSpeedErrorRpm()) <= SPEED_TOLERANCE_RPM;
    }

    /** Last output computed by NextMotor's event loop, not the velocity reference. */
    public double getAppliedPower() { return motor.getThrottle(); }

    /** Commands zero power immediately and preserves the driver's chosen RPM. */
    public void stop() {
        running = false;
        commandedRpm = 0.0;
        commandedPower = 0.0;
        motor.setThrottle(0.0);
    }
    public NextMotor hardware() {
        return motor;
    }
    public boolean isRunning() {
        return running;
    }

    /** Enables the hood servo and adjusts its position within [0, 1.0]. */
    public void changeHoodPosition(double change) {
        setHoodPosition(hoodPosition + change);
    }

    /** Commands an absolute normalized hood servo position. */
    public void setHoodPosition(double position) {
        if (!Double.isFinite(position)) throw new IllegalArgumentException("Hood position must be finite");
        hood.enable();
        hoodPosition = Math.max(0, Math.min(1.0, position));
        hood.setPosition(hoodPosition);
    }

    private static double clampRpm(double rpm) {
        return Math.max(0.0, Math.min(MAX_TARGET_RPM, rpm));
    }

    private static double slew(double current, double target, double maxChange) {
        return current + Math.max(-maxChange, Math.min(maxChange, target - current));
    }
}
