package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Locator;
import frc.robot.generated.TunerConstantsArkelon0416;
import frc.robot.subsystems.Drivetrain;

public class DriveTeleoperated extends Command {
    private double MaxSpeed = 1.0 * TunerConstantsArkelon0416.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentricFacingAngle drivePointing = new SwerveRequest.FieldCentricFacingAngle()
        .withHeadingPID(8, 0, 0)
        .withDeadband(MaxSpeed * 0.05)
        // .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt pointWheelsRobotForward = new SwerveRequest.PointWheelsAt()
        .withModuleDirection(Rotation2d.kZero);
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    Drivetrain drivetrain;
    Supplier<Double> xvelocity;
    Supplier<Double> yvelocity;
    Supplier<Double> omegavelocity;
    Supplier<Double> gasPedal;
    Supplier<Pose2d> hubPose;
    Trigger lockPoseTrigger;
    Trigger autoAlignTrigger;
    Trigger pointWheelsForwardTrigger;

    public DriveTeleoperated(Drivetrain drivetrain,
        Supplier<Double> xvelocity,
        Supplier<Double> yvelocity,
        Supplier<Double> omegavelocity,
        Supplier<Double> gasPedal,
        Supplier<Pose2d> hubPose,
        Trigger lockPoseTrigger,
        Trigger autoAlignTrigger,
        Trigger pointWheelsForwardTrigger
    ) {
        this.drivetrain = drivetrain;
        this.xvelocity = xvelocity;
        this.yvelocity = yvelocity;
        this.omegavelocity = omegavelocity;
        this.gasPedal = gasPedal;
        this.hubPose = hubPose;
        this.lockPoseTrigger = lockPoseTrigger;
        this.autoAlignTrigger = autoAlignTrigger;
        this.pointWheelsForwardTrigger = pointWheelsForwardTrigger;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(idle);
    }

    @Override
    public void execute() {
        SwerveRequest request;

        if (lockPoseTrigger.getAsBoolean()) {
            request = brake;
        } else if (pointWheelsForwardTrigger.getAsBoolean()) {
            request = pointWheelsRobotForward;
        } else if (autoAlignTrigger.getAsBoolean()) {
            // We are autoaligning, so use the pointing one
            Rotation2d target_angle = Rotation2d.fromRadians(
                Math.atan2(
                    drivetrain.getPos().getY()-this.hubPose.get().getY(),
                    drivetrain.getPos().getX()-this.hubPose.get().getX()
                )
            ).rotateBy(
                Locator.getInstance().hubPointFlipAngle
            );

            request = drivePointing
                .withVelocityX(-this.xvelocity.get() * MaxSpeed * this.gasPedal.get())
                .withVelocityY(-this.yvelocity.get() * MaxSpeed * this.gasPedal.get())
                .withTargetDirection(target_angle)
            ;
        } else {
            // Normal driving
            request = drive
                .withVelocityX(-this.xvelocity.get() * MaxSpeed * this.gasPedal.get())
                .withVelocityY(-this.yvelocity.get() * MaxSpeed * this.gasPedal.get())
                .withRotationalRate(-this.omegavelocity.get() * MaxAngularRate)
            ;
        }

        drivetrain.setControl(request);
    }
}