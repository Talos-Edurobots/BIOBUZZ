package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextServo;

public class Gate {
    public enum State { OPEN, CLOSED }

    private static final int SERVO_PORT = 2;
    private static final double OPEN_POSITION = 0;
    private static final double CLOSED_POSITION = 1;
    private static Gate instance;

    private final NextServo servo;
    private State state = State.CLOSED;
    private double targetPosition = CLOSED_POSITION;
    private boolean pwmEnabled = true;

    public static synchronized Gate get() {
        if (instance == null) {
            instance = new Gate();
        }
        return instance;
    }

    public Gate() {
        this(new NextServo(RobotController.expansionHub(), SERVO_PORT));
    }

    public Gate(NextServo servo) {
        if (servo == null) throw new IllegalArgumentException("servo must not be null");
        this.servo = servo;
    }

    public void activate() {
        setPosition(OPEN_POSITION);
        state = State.OPEN;
    }

    public void deactivate() {
        setPosition(CLOSED_POSITION);
        state = State.CLOSED;
    }

    public void toggle() {
        if (state == State.OPEN) deactivate();
        else activate();
    }

    public State getState() {
        return state;
    }

    public void enablePwm() {
        servo.enable();
        pwmEnabled = true;
        servo.setPosition(targetPosition);
    }

    public void disablePwm() {
        servo.disable();
        pwmEnabled = false;
    }

    public void setPosition(double position) {
        if (!Double.isFinite(position)) throw new IllegalArgumentException("position must be finite");
        targetPosition = Math.max(0.0, Math.min(1.0, position));
        if (pwmEnabled) servo.setPosition(targetPosition);
    }

    public void changePosition(double change) {
        setPosition(targetPosition + change);
    }
}
