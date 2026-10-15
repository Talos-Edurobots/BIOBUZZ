package org.firstinspires.ftc.teamcode.subsystems;

/** The robot's subsystems, shared by its OpMode. */
public class Robot {
    public final Drivetrain drivetrain = Drivetrain.get();
    public final Intake intake = Intake.get();
    public final Shooter shooter = Shooter.get();
    public final Turret turret = Turret.get();
    public final Gate gate = Gate.get();
    public final Flower flower = Flower.get();
    public final CRWheel crWheel = CRWheel.get();
}
