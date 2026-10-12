package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;

public class Shooter {
    private static final double RAMP_RATE_PER_SECOND = 0.5;
    private static Shooter instance;

    private final NextMotor motor;
    private final NextServo hood;
    private double hoodPosition = 0.5;

    private double targetPower = 0.0;
    private double currentPower = 0.0;
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
        this(new NextMotor(RobotController.expansionHub(), 2), new NextServo(RobotController.expansionHub(), 1));
    }

    public Shooter(NextMotor motor, NextServo hood) {
        this.motor = motor;
        this.hood = hood;
    }

    /** Starts ramping from zero toward the selected target power. */
    public void start() {
        running = true;
        currentPower = 0.0;
        lastUpdateNanos = System.nanoTime();
        motor.setThrottle(0.0);
    }

    /** Advances the power ramp once per OpMode loop. */
    public void update() {
        if (!running) {
            return;
        }

        long now = System.nanoTime();
        double elapsedSeconds = (now - lastUpdateNanos) / 1_000_000_000.0;
        lastUpdateNanos = now;

        double maxChange = RAMP_RATE_PER_SECOND * elapsedSeconds;
        if (currentPower < targetPower) {
            currentPower = Math.min(targetPower, currentPower + maxChange);
        } else {
            currentPower = Math.max(targetPower, currentPower - maxChange);
        }
        motor.setThrottle(currentPower);
    }

    /** Adjusts the ramp target; positive values increase it, negative values decrease it. */
    public void adjustTargetPower(double change) {
        targetPower = clampPower(targetPower + change);
    }

    public void setTargetPower(double power) {
        targetPower = clampPower(power);
    }

    public double getTargetPower() {
        return targetPower;
    }

    /** Returns the last power value commanded to the motor. */
    public double getAppliedPower() {
        return currentPower;
    }

    /** Stops immediately while preserving the selected target power. */
    public void stop() {
        running = false;
        currentPower = 0.0;
        motor.setThrottle(0.0);
    }
    public NextMotor hardware() {
        return motor;
    }
    public boolean isRunning() {
        return running;
    }

    /** Enables the hood servo and adjusts its position within [0.5, 1.0]. */
    public void changeHoodPosition(double change) {
        hood.enable();
        hoodPosition = Math.max(0.5, Math.min(1.0, hoodPosition + change));
        hood.setPosition(hoodPosition);
    }

    private double clampPower(double power) {
        return Math.max(0.0, Math.min(1.0, power));
    }
}
