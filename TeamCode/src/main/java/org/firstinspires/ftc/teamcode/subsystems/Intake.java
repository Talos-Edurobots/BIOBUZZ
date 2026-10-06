package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {
    private final NextMotor motor = new NextMotor("intake");
    private double currentPower = 0.0;

    /**
     * Sets the intake power once. The motor keeps that power until another
     * command changes it or the OpMode ends.
     */
    public Command intake(double power) {
        double safePower = clampMagnitude(power);
        return instant(() -> {
            currentPower = safePower;
            motor.setThrottle(currentPower);
        });
    }

    /** Sets reverse power once; it remains set until another command changes it. */
    public Command outtake(double power) {
        double safePower = clampMagnitude(power);
        return instant(() -> {
            currentPower = -safePower;
            motor.setThrottle(currentPower);
        });
    }

    /** Stops the motor once this command runs. */
    public Command stop() {
        return instant(() -> {
            currentPower = 0.0;
            motor.setThrottle(currentPower);
        });
    }

    /** Returns the last power value commanded to the motor. */
    public double getCurrentPower() {
        return currentPower;
    }

    private double clampMagnitude(double power) {
        return Math.max(0.0, Math.min(1.0, power));
    }
}
