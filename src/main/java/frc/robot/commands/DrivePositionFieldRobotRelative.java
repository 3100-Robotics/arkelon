package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DrivePositionFieldRobotRelative extends Command {
    private Pose2d targetPosition;
    private Supplier<Pose2d> targetSupplier;
    private Drivetrain drivetrain;

    public DrivePositionFieldRobotRelative(Drivetrain drivetrain, Supplier<Pose2d> targetSupplier) {
        this.targetSupplier = targetSupplier;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.goToPose();
        if (drivetrain.isAtPoseSetpoint(false))
            { cancel(); }
    }

    @Override
    public void end(boolean interupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }

    @Override
    public void initialize() {
        targetPosition = targetSupplier.get();
        drivetrain.poseSetpoint = () -> targetPosition;
    }
}
