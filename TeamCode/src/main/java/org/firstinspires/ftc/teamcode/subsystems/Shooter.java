package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Shooter implements Mechanism {
    private final NextMotor motor = new NextMotor("shooter");

    private double targetPower = 0.0;
    private double currentPower = 0.0;
    private long lastUpdateNanos = System.nanoTime();

    /**
     * Creates a command that continuously ramps toward the current target power
     * and holds it there. Rate is measured in power units per second.
     */
    public Command runAtTarget(double ratePerSecond) {
        if (ratePerSecond <= 0.0) {
            throw new IllegalArgumentException("ratePerSecond must be greater than zero");
        }

        return infinite(() -> {
            long now = System.nanoTime();
            double elapsedSeconds = (now - lastUpdateNanos) / 1_000_000_000.0;
            lastUpdateNanos = now;

            double target = targetPower;
            double maxChange = ratePerSecond * elapsedSeconds;
            if (currentPower < target) {
                currentPower = Math.min(target, currentPower + maxChange);
            } else {
                currentPower = Math.max(target, currentPower - maxChange);
            }

            motor.setThrottle(currentPower);
        });
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

    /** Stops the shooter immediately while preserving its selected target power. */
    public Command stop() {
        return instant(() -> {
            currentPower = 0.0;
            lastUpdateNanos = System.nanoTime();
            motor.setThrottle(0.0);
        });
    }


    @Override
    public Command getDefaultCommand() {
        return infinite(() -> {
            currentPower = 0.0;
            lastUpdateNanos = System.nanoTime();
            motor.setThrottle(0.0);
        });
    }

    private double clampPower(double power) {
        return Math.max(0.0, Math.min(1.0, power));
    }
}
