package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextCRServo;

public class CRWheel {
    private static final int SERVO_PORT = 4;
    private static CRWheel instance;

    private final NextCRServo servo;

    public static synchronized CRWheel get() {
        if (instance == null) {
            instance = new CRWheel();
        }
        return instance;
    }

    public CRWheel() {
        this(new NextCRServo(RobotController.controlHub(), SERVO_PORT));
    }

    public CRWheel(NextCRServo servo) {
        if (servo == null) throw new IllegalArgumentException("servo must not be null");
        this.servo = servo;
    }

    public void setPower(double power) {
        if (!Double.isFinite(power)) throw new IllegalArgumentException("power must be finite");
        servo.setPower(Math.max(-1.0, Math.min(1.0, power)));
    }

    public void stop() {
        setPower(0.0);
    }
}
