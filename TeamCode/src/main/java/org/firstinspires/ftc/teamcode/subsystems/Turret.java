package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextServo;

public class Turret {
    private static Turret instance;

    private final NextServo servo;
    private double targetPosition = 0.5;
    private boolean active = true;

    public static synchronized Turret get() {
        if (instance == null) {
            instance = new Turret();
        }
        return instance;
    }

    public static synchronized void setInstance(Turret turret) {
        instance = turret;
    }

    public Turret() {
        this(new NextServo(RobotController.controlHub(), 1));
    }

    public Turret(NextServo servo) {
        this.servo = servo;
    }

    /** Enables PWM and commands the servo to its last selected position. */
    public void activate() {
        servo.enable();
        active = true;
        servo.setPosition(targetPosition);
    }

    /** Disables the servo's PWM signal. It may no longer hold its position. */
    public void deactivate() {
        servo.disable();
        active = false;
    }

    /** Sets a target position in the range [0.0, 1.0]. */
    public void setPosition(double position) {
        targetPosition = clampPosition(position);
        servo.setPosition(targetPosition);
    }

    public void adjustPosition(double change) {
        setPosition(targetPosition + change);
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
