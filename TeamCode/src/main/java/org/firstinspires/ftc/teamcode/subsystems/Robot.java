package org.firstinspires.ftc.teamcode.subsystems;

/** The robot's subsystems, shared by its OpMode. */
public class Robot {
    public final Drivetrain drivetrain = Drivetrain.get();
    public final Intake intake = Intake.get();
    public final Shooter shooter = Shooter.get();
    public final Turret turret = Turret.get();
}
