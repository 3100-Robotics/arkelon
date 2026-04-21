package frc.robot.commands;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import edu.wpi.first.wpilibj.util.WPILibVersion;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Locator;
import frc.robot.RobotContainer;
import frc.robot.Vision;
import frc.robot.States.Indexer.TripleRollerStates;
import frc.robot.States.Intake.PivotState;
import frc.robot.States.Intake.RollerState;
import frc.robot.States.Shooter.FlywheelStates;
import frc.robot.States.Shooter.HoodState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Utils.Direction;

@SuppressWarnings("unused")
public class Auton {
    private AutoChooser autoChooser = new AutoChooser();
    private AutoFactory autoFactory;

    // Subsystem refs
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Indexer indexer;
    private Intake intake;
    private Vision vision;
    private RobotContainer rcontainer;

    public Trigger astop = new Trigger(() -> SmartDashboard.getBoolean("astop",
                        false
                    ));

    public Auton(
        Drivetrain drivetrain,
        Shooter shooter,
        Indexer indexer,
        Intake intake,
        RobotContainer rcontainer,
        Vision vision
    ) {
        this.rcontainer = rcontainer;
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        this.vision = vision;

        autoFactory = new AutoFactory(
            this.drivetrain::getPos,
            this.drivetrain::resetPose,
            this.drivetrain::followTrajectory,
            true,
            this.drivetrain
        );

        autoChooser.addCmd("Do nothing", Commands::none);
        autoChooser.addRoutine("xbump_right", () -> crossBump(Direction.Right));
        autoChooser.addRoutine("xbump_left", () -> crossBump(Direction.Left));
        autoChooser.addRoutine("outpost", this::outpost);

        SmartDashboard.putData("Auton Selector", autoChooser);
        SmartDashboard.putBoolean("astop", false);
        RobotModeTriggers.autonomous()
            .whileTrue(autoChooser.selectedCommandScheduler()
                .unless(astop)
        );

        RobotModeTriggers.teleop()
            .or(astop)
            .onTrue(
                Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll())
                    .alongWith(Commands.runOnce(() -> vision.usePose = true))
            );
    }

    public AutoRoutine crossBump(Direction side) {
        var routine = autoFactory.newRoutine("xbump_"+side.toString());

        var part1 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("xbump_part1"));
        var part2 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("xbump_part2"));
        var part3 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("xbump_part3"));

        routine.active().onTrue(
            Commands.sequence(
                part1.resetOdometry(),
                intake.setPivotState(PivotState.Medium),
                part1.cmd(),
                drivetrain.goToPoseCommand(() -> part2.getInitialPose().get())
                    .until(drivetrain.isAtPoseSetpoint),
                Commands.sequence(
                    intake.setPivotState(PivotState.FullDeploy),
                    intake.setRollerState(RollerState.Off)
                ),
                part2.cmd()
            )
        );

        part2.inactive().onTrue(
            Commands.sequence(
                Commands.parallel(
                    rcontainer.shootDialed(),
                    Commands.waitSeconds(1)
                        .andThen(intake.setPivotState(PivotState.FullDeploy))
                        .andThen(intake.setRollerState(RollerState.On))
                ),
                Commands.waitSeconds(5),
                Commands.sequence(
                    intake.setPivotState(PivotState.Medium),
                    intake.setRollerState(RollerState.Off),
                    indexer.setState(TripleRollerStates.Off),
                    shooter.setFlywheelState(FlywheelStates.Frozen),
                    shooter.setHoodState(HoodState.Frozen)
                ),
                part3.cmd()
            )
        );

        part1.atTime("DeployIntake").onTrue(
            Commands.sequence(
                intake.setPivotState(PivotState.FullDeploy),
                intake.setRollerState(RollerState.On)
            )
        );

        return routine;
    }

    public AutoRoutine outpost() {
        var routine = autoFactory.newRoutine("outpost");

        var part1 = routine.trajectory("outpost_part1");
        var part2 = routine.trajectory("outpost_part2");

        routine.active().onTrue(
            Commands.sequence(
                part1.resetOdometry(),
                intake.setPivotState(PivotState.FullDeploy),
                part1.cmd(),
                // Collect

                Commands.waitSeconds(5),
                drivetrain.goToPoseCommand(part2.getInitialPose()::get).withTimeout(0.3),
                part2.cmd(),
                Commands.parallel(
                    drivetrain.pointAtPose(() -> Locator.getInstance().hubPose),
                    rcontainer.shootDialed()
                )
            )
        );

        return routine;
    }

    public AutoRoutine outpostNeutralZone() {
        var routine = autoFactory.newRoutine("outpostNeutralZone");

        var part1 = routine.trajectory("outpost_part1");
        var part2 = routine.trajectory("outpost_part2");

        routine.active().onTrue(
            Commands.sequence(
                part1.resetOdometry(),
                intake.setPivotState(PivotState.FullDeploy),
                part1.cmd(),
                // Collect
                Commands.waitSeconds(5),
                drivetrain.goToPoseCommand(part2.getInitialPose()::get),
                part2.cmd(),
                Commands.parallel(
                    drivetrain.pointAtPose(() -> Locator.getInstance().hubPose),
                    rcontainer.shootDialed()
                )
            )
        );

        return routine;
    }

}
