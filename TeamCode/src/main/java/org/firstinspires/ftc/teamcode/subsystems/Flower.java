package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextServo;

public class Flower {
    public enum State { ACTIVE, INACTIVE }

    private static final int SERVO_PORT = 3;
    private static final double ACTIVE_POSITION = 0.6;
    private static final double INACTIVE_POSITION = 0.4;
    private static Flower instance;

    private final NextServo servo;
    private State state = State.INACTIVE;
    private double targetPosition = INACTIVE_POSITION;
    private boolean pwmEnabled = true;

    public static synchronized Flower get() {
        if (instance == null) {
            instance = new Flower();
        }
        return instance;
    }

    public Flower() {
        this(new NextServo(RobotController.controlHub(), SERVO_PORT));
    }

    public Flower(NextServo servo) {
        if (servo == null) throw new IllegalArgumentException("servo must not be null");
        this.servo = servo;
    }

    public void activate() {
        setPosition(ACTIVE_POSITION);
        state = State.ACTIVE;
    }

    public void deactivate() {
        setPosition(INACTIVE_POSITION);
        state = State.INACTIVE;
    }

    public void toggle() {
        if (state == State.ACTIVE) deactivate();
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
