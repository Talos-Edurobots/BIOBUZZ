package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.drive.DriveCommands;

public class Drivetrain implements Mechanism {
    public final NextMotor frontLeft = new NextMotor(RobotController.expansionHub(), 1);
    public final NextMotor frontRight = new NextMotor(RobotController.controlHub(), 2);
    public final NextMotor backLeft = new NextMotor(RobotController.expansionHub(), 0);
    public final NextMotor backRight = new NextMotor(RobotController.controlHub(), 1);

    /** Starts the mecanum drive command, which keeps reading this gamepad each loop. */
    public void startDrive(Gamepad gamepad) {
        DriveCommands.mecanumDrive(frontLeft, frontRight, backLeft, backRight, gamepad).schedule();
    }
}
