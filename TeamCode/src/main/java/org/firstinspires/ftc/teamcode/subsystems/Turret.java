package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class Turret implements Mechanism {
    private final NextServo servo = new NextServo("turret");
    private double targetPosition = 0.5;
    private boolean active = true;

    /** Enables PWM and commands the servo to its last selected position. */
    public Command activate() {
        return instant(() -> {
            servo.enable();
            active = true;
            servo.setPosition(targetPosition);
        });
    }

    /** Disables the servo's PWM signal. It may no longer hold its position. */
    public Command deactivate() {
        return instant(() -> {
            servo.disable();
            active = false;
        });
    }

    /** Sets a target position in the range [0.0, 1.0]. */
    public Command setPosition(double position) {
        final double safePosition = clampPosition(position);
        return instant(() -> {
            targetPosition = safePosition;
            servo.setPosition(safePosition);
        });
    }
    public void adjustPosition(double change) {
        targetPosition = clampPosition(targetPosition + change);
        servo.setPosition(targetPosition);
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    /** Returns whether this subsystem has enabled the servo's PWM output. */
    public boolean isActive() {
        return active;
    }

    private double clampPosition(double position) {
        return Math.max(0.0, Math.min(1.0, position));
    }
}
