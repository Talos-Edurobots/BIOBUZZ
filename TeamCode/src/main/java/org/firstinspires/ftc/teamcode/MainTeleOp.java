package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.commands.Commands;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@NextTeleop(name = "Main TeleOp")
public class MainTeleOp extends NextOpMode {
    private final Robot robot;

    public MainTeleOp(Robot robot) {
        super(robot);
        this.robot = robot;
    }

    @Override
    public void start() {
        robot.drivetrain.startDrive(gamepad1);

        CommandGamepad driver = new CommandGamepad(gamepad1);
        driver.a().onTrue(robot.intake.intake(1));
        driver.b().onTrue(robot.intake.stop());
        driver.y().onTrue(robot.intake.outtake(1));

        driver.x().toggleOnTrue(robot.shooter.runAtTarget(0.5));
        driver.dpadUp().onTrue(
                Commands.instant(() -> robot.shooter.adjustTargetPower(0.1))
        );
        driver.dpadDown().onTrue(
                Commands.instant(() -> robot.shooter.adjustTargetPower(-0.1))
        );

        driver.dpadLeft().onTrue(
                Commands.instant(() -> robot.turret.adjustPosition(-0.1))
        );
        driver.dpadRight().onTrue(
                Commands.instant(() -> robot.turret.adjustPosition(0.1))
        );
        driver.start().onTrue(robot.turret.activate());
        driver.back().onTrue(robot.turret.deactivate());
    }

    @Override
    public void periodic() {
        telemetry.addLine("Controls: A intake | B stop intake | Y outtake | X shooter on/off | D-pad Up/Down shooter target | D-pad Left/Right turret position | Start/Back turret PWM on/off");
        telemetry.addData("Shooter target power", "%.2f", robot.shooter.getTargetPower());
        telemetry.addData("Shooter applied power", "%.2f", robot.shooter.getAppliedPower());
        telemetry.addData("Turret commanded position", "%.2f", robot.turret.getTargetPosition());
        telemetry.addData("Turret PWM active", robot.turret.isActive());
        telemetry.addData("Intake power", "%.2f", robot.intake.getCurrentPower());
    }
}
