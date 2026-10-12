package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import dev.nextftc.control.drive.DriveInput;
import dev.nextftc.control.drive.MecanumKinematics;
import dev.nextftc.control.drive.MecanumWheelPowers;
import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;

public class Drivetrain {
    private static Drivetrain instance;

    public final NextMotor frontLeft;
    public final NextMotor frontRight;
    public final NextMotor backLeft;
    public final NextMotor backRight;

    private final MecanumKinematics kinematics = new MecanumKinematics();

    public static synchronized Drivetrain get() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }

    public static synchronized void setInstance(Drivetrain drivetrain) {
        instance = drivetrain;
    }

    public Drivetrain() {
        this(
            new NextMotor(RobotController.expansionHub(), 1),
            new NextMotor(RobotController.controlHub(), 2),
            new NextMotor(RobotController.expansionHub(), 0),
            new NextMotor(RobotController.controlHub(), 1)
        );
    }

    public Drivetrain(NextMotor frontLeft, NextMotor frontRight, NextMotor backLeft, NextMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    /** Reads the sticks and sets all four motor powers once per OpMode loop. */
    public void drive(Gamepad gamepad) {
        MecanumWheelPowers powers = kinematics.calculate(new DriveInput(
                -gamepad.left_stick_y,
                gamepad.left_stick_x,
                gamepad.right_stick_x
        ));
        frontLeft.setThrottle(powers.getFrontLeft());
        frontRight.setThrottle(powers.getFrontRight());
        backLeft.setThrottle(powers.getBackLeft());
        backRight.setThrottle(powers.getBackRight());
    }

    public void stop() {
        frontLeft.setThrottle(0.0);
        frontRight.setThrottle(0.0);
        backLeft.setThrottle(0.0);
        backRight.setThrottle(0.0);
    }
}
