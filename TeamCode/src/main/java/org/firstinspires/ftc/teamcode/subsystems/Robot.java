package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;

public class Robot implements NextRobot {
    public final Drivetrain drivetrain = new Drivetrain();
    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final Turret turret = new Turret();

    private final Set<Mechanism> mechanisms = Collections.unmodifiableSet(
            new LinkedHashSet<>(Arrays.asList(drivetrain, intake, shooter, turret))
    );

    @Override
    public Set<Mechanism> getMechanisms() {
        return mechanisms;
    }
}
