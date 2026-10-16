package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.hardware.actuators.NextMotor;

import org.firstinspires.ftc.teamcode.data.StructuredRobotDataClient;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends OpMode {
    private Robot robot;
    private StructuredRobotDataClient client;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        robot = new Robot();
        client = new StructuredRobotDataClient("Main TeleOp")
                .addRuntime(runtime)
                .addGamepads(gamepad1, gamepad2)
                .addMotor("intake", "Intake Motor", robot.intake.hardware(), robot.intake::getCurrentPower)
                .addMotor("shooter", "Shooter Motor", robot.shooter.hardware(), robot.shooter::getAppliedPower)
                .addMotor("drivetrain.frontLeft", "Front Left Motor", robot.drivetrain.frontLeft)
                .addMotor("drivetrain.frontRight", "Front Right Motor", robot.drivetrain.frontRight)
                .addMotor("drivetrain.backLeft", "Back Left Motor", robot.drivetrain.backLeft)
                .addMotor("drivetrain.backRight", "Back Right Motor", robot.drivetrain.backRight);
        client.start();
    }

    @Override
    public void start() {
        robot.turret.activate();
    }

    @Override
    public void loop() {
        robot.drivetrain.drive(gamepad1);

        if (gamepad1.aWasPressed()) {
            robot.intake.intake(1.0);
            robot.crWheel.setPower(1.0);
        }
        if (gamepad1.yWasPressed()) {
            robot.intake.outtake(1.0);
            robot.crWheel.setPower(-1.0);
        }
        if (gamepad1.bWasPressed()) {
            robot.intake.stop();
            robot.crWheel.stop();
        }
        if (gamepad1.rightTriggerWasPressed()) robot.gate.toggle();

        if (gamepad1.xWasPressed()) {
            if (robot.shooter.isRunning()) {
                robot.shooter.stop();
            } else {
                robot.shooter.start();
            }
        }
        if (gamepad1.dpadUpWasPressed()) robot.shooter.adjustTargetPower(0.1);
        if (gamepad1.dpadDownWasPressed()) robot.shooter.adjustTargetPower(-0.1);

        if (gamepad1.dpadLeftWasPressed()) robot.turret.adjustPosition(-0.1);
        if (gamepad1.dpadRightWasPressed()) robot.turret.adjustPosition(0.1);
        if (gamepad1.startWasPressed()) robot.turret.activate();
        if (gamepad1.backWasPressed()) robot.turret.deactivate();

        if (gamepad1.leftBumperWasPressed()) robot.shooter.changeHoodPosition(-0.1);
        if (gamepad1.rightBumperWasPressed()) robot.shooter.changeHoodPosition(0.1);

        robot.shooter.update();
        // NextMotor applies the requested powers when its motor loop is polled.
        NextMotor.Companion.getMotorEventLoop().poll();

        telemetry.addLine("Controls: A intake + wheel | B stop intake + wheel | Y outtake + wheel | Right trigger gate | X shooter on/off | D-pad Up/Down shooter target | D-pad Left/Right turret position | Start/Back turret PWM on/off | Bumpers hood");
        telemetry.addData("Shooter target power", "%.2f", robot.shooter.getTargetPower());
        telemetry.addData("Shooter applied power", "%.2f", robot.shooter.getAppliedPower());
        telemetry.addData("Turret commanded position", "%.2f", robot.turret.getTargetPosition());
        telemetry.addData("Turret PWM active", robot.turret.isActive());
        telemetry.addData("Intake power", "%.2f", robot.intake.getCurrentPower());
        telemetry.addData("Gate state", robot.gate.getState());
        telemetry.addData("Intake current", "%.2f A",
                robot.intake.hardware().getCurrent().getMagnitude());
        telemetry.addData("Shooter current", "%.2f A",
                robot.shooter.hardware().getCurrent().getMagnitude());
        telemetry.update();

        client.publishLoop();
    }

    @Override
    public void stop() {
        robot.drivetrain.stop();
        robot.intake.stop();
        robot.crWheel.stop();
        robot.shooter.stop();
        NextMotor.Companion.getMotorEventLoop().poll();
        if (client != null) {
            client.close();
        }
    }
}
