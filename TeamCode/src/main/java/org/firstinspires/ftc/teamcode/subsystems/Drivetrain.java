package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.drive.DriveCommands;

public class Drivetrain implements Mechanism {
    public final NextMotor frontLeft = new NextMotor("frontLeft");
    public final NextMotor frontRight = new NextMotor("frontRight");
    public final NextMotor backLeft = new NextMotor("backLeft");
    public final NextMotor backRight = new NextMotor("backRight");

    /** Starts the mecanum drive command, which keeps reading this gamepad each loop. */
    public void startDrive(Gamepad gamepad) {
        DriveCommands.mecanumDrive(frontLeft, frontRight, backLeft, backRight, gamepad).schedule();
    }
}
