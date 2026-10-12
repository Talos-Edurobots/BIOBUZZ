package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;

public class Intake {
    private static Intake instance;

    private final NextMotor motor;
    private double currentPower = 0.0;

    public static synchronized Intake get() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public static synchronized void setInstance(Intake intake) {
        instance = intake;
    }

    public Intake() {
        this(new NextMotor(RobotController.controlHub(), 0));
    }

    public Intake(NextMotor motor) {
        this.motor = motor;
    }

    /** Sets intake power until another action changes it. */
    public void intake(double power) {
        setPower(clampMagnitude(power));
    }
    public NextMotor hardware() {
        return motor;
    }

    /** Sets reverse power until another action changes it. */
    public void outtake(double power) {
        setPower(-clampMagnitude(power));
    }

    public void stop() {
        setPower(0.0);
    }

    /** Returns the last power value commanded to the motor. */
    public double getCurrentPower() {
        return currentPower;
    }

    public NextMotor hardware() {
        return motor;
    }

    private void setPower(double power) {
        currentPower = power;
        motor.setThrottle(power);
    }

    private double clampMagnitude(double power) {
        return Math.max(0.0, Math.min(1.0, power));
    }
}
