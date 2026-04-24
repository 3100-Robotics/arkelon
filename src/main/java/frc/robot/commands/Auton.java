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
import frc.robot.MatchConfig;
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

        autoChooser.addRoutine("xbump2_right", () -> crossBump2(Direction.Right));
        autoChooser.addRoutine("xbump2_left", () -> crossBump2(Direction.Left));

        autoChooser.addRoutine("outpost", this::outpost);
        autoChooser.addRoutine("outpostNeutralZone", this::outpostNeutralZone);

        SmartDashboard.putData("Auton Selector", autoChooser);
        SmartDashboard.putBoolean("astop", false);
        RobotModeTriggers.autonomous()
            .whileTrue(
                Commands.sequence(
                    Commands.waitSeconds(MatchConfig.autonDelay),
                    autoChooser.selectedCommandScheduler()
                )
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
                    .withTimeout(0.5),
                drivetrain.goToPoseCommand(() -> part2.getInitialPose().get())
                    .until(drivetrain.isAtPoseSetpoint),
                Commands.sequence(
                    intake.setPivotState(PivotState.FullDeploy),
                    intake.setRollerState(RollerState.Off)
                ),
                part2.cmd(),
                Commands.parallel(
                    rcontainer.shootDialed(),
                    Commands.waitSeconds(1)
                        .andThen(intake.setPivotState(PivotState.Medium))
                        .andThen(intake.setRollerState(RollerState.On)),
                    drivetrain.pointAtPose(() -> Locator.getInstance().hubPose)
                ).withTimeout(5),
                Commands.sequence(
                    intake.setPivotState(PivotState.Medium),
                    intake.setRollerState(RollerState.Off),
                    indexer.setState(TripleRollerStates.Off),
                    shooter.setFlywheelState(FlywheelStates.Frozen),
                    shooter.setHoodState(HoodState.Frozen)
                ),
                part3.cmd(),
                Commands.runOnce(() -> drivetrain.setControl(new SwerveRequest.Idle()))
            )
        );

        part1.atTime("DeployIntake").onTrue(
            Commands.sequence(
                intake.setPivotState(PivotState.FullDeploy),
                intake.setRollerState(RollerState.On)
            )
        );

        part3.atTime("Collect").onTrue(
            Commands.sequence(
                intake.setPivotState(PivotState.FullDeploy),
                intake.setRollerState(RollerState.On)
            )
        );

        return routine;
    }

    public AutoRoutine crossBump2(Direction side) {
        var routine = autoFactory.newRoutine("xbump_"+side.toString());

        var part1 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("xbump_part1"));
        var part2 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("xbump_part2"));
        var part3_v2 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("xbump_part3_v2"));

        routine.active().onTrue(
            Commands.sequence(
                part1.resetOdometry(),
                intake.setPivotState(PivotState.Medium),
                part1.cmd(),
                drivetrain.goToPoseCommand(() -> part2.getInitialPose().get())
                    .withTimeout(0.5),
                drivetrain.goToPoseCommand(() -> part2.getInitialPose().get())
                    .until(drivetrain.isAtPoseSetpoint),
                Commands.sequence(
                    intake.setPivotState(PivotState.FullDeploy),
                    intake.setRollerState(RollerState.Off)
                ),
                part2.cmd(),
                Commands.parallel(
                    rcontainer.shootDialed(),
                    Commands.waitSeconds(1)
                        .andThen(intake.setPivotState(PivotState.Medium))
                        .andThen(intake.setRollerState(RollerState.On)),
                    drivetrain.pointAtPose(() -> Locator.getInstance().hubPose)
                ).withTimeout(5),
                Commands.sequence(
                    intake.setPivotState(PivotState.Medium),
                    intake.setRollerState(RollerState.Off),
                    indexer.setState(TripleRollerStates.Off),
                    shooter.setFlywheelState(FlywheelStates.Frozen),
                    shooter.setHoodState(HoodState.Frozen)
                ),
                part3_v2.cmd(),
                Commands.runOnce(() -> drivetrain.setControl(new SwerveRequest.Idle())),
                Commands.parallel(
                    rcontainer.shootDialed(),
                    Commands.waitSeconds(1)
                        .andThen(intake.setPivotState(PivotState.Medium))
                        .andThen(intake.setRollerState(RollerState.On)),
                    drivetrain.pointAtPose(() -> Locator.getInstance().hubPose)
                ).withTimeout(10)
            )
        );

        part1.atTime("DeployIntake").onTrue(
            Commands.sequence(
                intake.setPivotState(PivotState.FullDeploy),
                intake.setRollerState(RollerState.On)
            )
        );

        part3_v2.atTime("Collect").onTrue(
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
                Commands.runOnce(() -> drivetrain.setControl(new SwerveRequest.Idle())),
                // Collect

                Commands.waitSeconds(4),
                drivetrain.goToPoseCommand(part2.getInitialPose()::get).withTimeout(0.2),
                part2.cmd(),
                Commands.parallel(
                    Commands.waitSeconds(1)
                        .andThen(intake.setRollerState(RollerState.On))
                        .andThen(intake.setPivotState(PivotState.Medium)
                    ),
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
        var part3 = routine.trajectory("outpost_part3");

        routine.active().onTrue(
            Commands.sequence(
                part1.resetOdometry(),
                intake.setPivotState(PivotState.FullDeploy),
                part1.cmd(),
                Commands.runOnce(() -> drivetrain.setControl(new SwerveRequest.Idle())),
                // Collect

                Commands.waitSeconds(4),
                drivetrain.goToPoseCommand(part2.getInitialPose()::get).withTimeout(0.2),
                part2.cmd(),
                Commands.parallel(
                    Commands.waitSeconds(1)
                        .andThen(intake.setRollerState(RollerState.On))
                        .andThen(intake.setPivotState(PivotState.Medium)
                    ),
                    drivetrain.pointAtPose(() -> Locator.getInstance().hubPose),
                    rcontainer.shootDialed()
                ).withTimeout(5),
                rcontainer.postShootIdles()
                    .andThen(intake.setRollerState(RollerState.Off))
                    .withTimeout(0),
                drivetrain.goToPoseCommand(part3.getInitialPose()::get)
                    .withTimeout(0.2),
                part3.cmd()
            )
        );

        return routine;
    }

    // Segments
    public Command outpostPart1(AutoRoutine routine) {
        var part1 = routine.trajectory("outpost_part1");
        var part2 = routine.trajectory("outpost_part2");

        return Commands.sequence(
            part1.resetOdometry(),
            intake.setPivotState(PivotState.FullDeploy),
            part1.cmd(),
            Commands.runOnce(() -> drivetrain.setControl(new SwerveRequest.Idle())),
            // Collect

            Commands.waitSeconds(4),
            drivetrain.goToPoseCommand(part2.getInitialPose()::get).withTimeout(0.2),
            part2.cmd(),
            Commands.parallel(
                Commands.waitSeconds(1)
                    .andThen(intake.setRollerState(RollerState.On))
                    .andThen(intake.setPivotState(PivotState.Medium)
                ),
                drivetrain.pointAtPose(() -> Locator.getInstance().hubPose),
                rcontainer.shootDialed()
            )
        );
    }

    public Command outpostPart2(AutoRoutine routine) {
        var part3 = routine.trajectory("outpost_part3");
        
        return Commands.sequence(
            rcontainer.postShootIdles()
                .andThen(intake.setRollerState(RollerState.Off))
                .withTimeout(0),
            drivetrain.goToPoseCommand(part3.getInitialPose()::get)
                .withTimeout(0.2),
            part3.cmd()
        );
    }
}
