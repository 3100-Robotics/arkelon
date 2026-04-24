// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.States.Indexer.TripleRollerStates;
import frc.robot.States.Intake.RollerState;
import frc.robot.States.Shooter.FlywheelStates;
import frc.robot.States.Shooter.HoodState;
import frc.robot.commands.Auton;
import frc.robot.commands.DriveTeleoperated;
import frc.robot.generated.TunerConstantsArkelon0416;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private static RobotContainer instance;
    public static RobotContainer getInstance() {
        if (instance==null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private final CommandXboxController brendanCtl = new CommandXboxController(0);
    private final CommandXboxController jesusCtl = new CommandXboxController(1);

    public Distance shooterDistance = Meters.of(0);
    public Supplier<Distance> getShooterDistance = () -> shooterDistance;

    private Intake intake = new Intake(); 
    private Indexer indexer = new Indexer();
    public Shooter shooter = new Shooter(() -> Shooter.calculateHoodAngle(getShooterDistance.get()), 
                                          () -> Shooter.calculateFlywheelSpeed(getShooterDistance.get())
                                        );
    private Drivetrain drivetrain = TunerConstantsArkelon0416.createDrivetrain();
    
    public Locator locator;
    private Vision vision;

    public Auton auto;

    public RobotContainer() {
        instance = this;
        locator = new Locator(drivetrain::getPos, drivetrain);
        vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::getPos);

        drivetrain.registerTelemetry(Logging.getInstance()::logCTREChassis);

        Robot.getInstance().addPeriodic(this::calculateShooterDistance, 0.02);
        
        auto = new Auton(drivetrain, shooter, indexer, intake, this, vision);
        configureBindings();
    }

    public void calculateShooterDistance() {
        shooterDistance = Locator.getInstance().getDistanceToHub() // Robot center distance from hub
            .plus(Inches.of(5.4330709)) // Center to fuel exit
        ;
    }

    public Command shootDialed() {
        return shootDialed(HoodState.Varying, FlywheelStates.Varying);
    }

    public Command shootDialed(HoodState hoodState, FlywheelStates flywheelStates) {
        return Commands.sequence(
            shooter.setFlywheelState(flywheelStates),
            shooter.setHoodState(hoodState),
            Commands.waitSeconds(0.1).andThen(Commands.waitUntil(shooter.flywheelsAtAccel)),
            indexer.setState(TripleRollerStates.On)
        );
    }

    public Command postShootIdles() {
        return Commands.parallel(
            shooter.setFlywheelState(FlywheelStates.Frozen),
            shooter.setHoodState(HoodState.Frozen),
            indexer.setState(TripleRollerStates.Off)
        );
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            new DriveTeleoperated(drivetrain,
                brendanCtl::getLeftY,
                brendanCtl::getLeftX,
                brendanCtl::getRightX,
                brendanCtl::getRightTriggerAxis,
                () -> Locator.getInstance().hubPose,
                brendanCtl.b(),
                brendanCtl.a(),
                brendanCtl.leftStick()
            )
        );

        // Brendan shoot is: X, Jesus is A
        brendanCtl.x().or(jesusCtl.a()).or(brendanCtl.rightStick()).whileTrue(
            shootDialed()
        ).whileFalse(
            Commands.sequence(
                shooter.setFlywheelState(FlywheelStates.Frozen),
                shooter.setHoodState(HoodState.Frozen),
                indexer.setState(TripleRollerStates.Off)
            )
        );

        // Jesus povLeft reverse indexer
        jesusCtl.povLeft()
            .whileTrue(indexer.setState(TripleRollerStates.Reverse));
        
        // Brendan intake up/down NORMAL is: leftBumper
        brendanCtl.leftBumper().or(jesusCtl.leftBumper())
            .whileTrue(intake.togglePivot());
        jesusCtl.leftTrigger().or(brendanCtl.y()).whileTrue(intake.setFullStow()).whileFalse(intake.leaveFullStow());
        
        // Brendan rin intake: left trigger
        brendanCtl.leftTrigger().or(jesusCtl.rightBumper())
            .whileTrue(intake.setRollerState(RollerState.On))
            .whileFalse(intake.setRollerState(RollerState.Off))
        ;
        
        // Idle all: povUp
        brendanCtl.povUp().or(jesusCtl.povUp()).whileTrue(
            Commands.parallel(
                intake.setRollerState(RollerState.Off),
                indexer.setState(TripleRollerStates.Off),
                shooter.setFlywheelState(FlywheelStates.Frozen),
                shooter.setHoodState(HoodState.Reset)
            )
        );
        
        // Reset odometry: povDown
        brendanCtl.povDown()
            .whileTrue(Commands.runOnce(() -> drivetrain.resetPose(Locator.getInstance().towerPose)));
        
        // Toggle vision: povRight
        brendanCtl.povRight().whileTrue(Commands.runOnce(() -> {
            if (vision.usePose == true) {
                vision.usePose = false;
            } else {
                vision.usePose = true;
            }
        }));

        // Keep flywheel spun up while holding: rightBumper
        brendanCtl.rightBumper()
            .whileTrue(shooter.setFlywheelState(FlywheelStates.Varying))
            .whileFalse(shooter.setFlywheelState(FlywheelStates.Frozen))
        ;

        // Reverse intake povLeft
        brendanCtl.povLeft()
            .whileTrue(intake.setRollerState(RollerState.Reverse))
            .whileFalse(intake.setRollerState(RollerState.Off))
        ;

        jesusCtl.x()
            .whileTrue(shootDialed(HoodState.OpposeToAlly, FlywheelStates.OpposeToAlly))
            .whileFalse(postShootIdles())
        ;

        jesusCtl.y() 
            .whileTrue(shootDialed(HoodState.HubShot, FlywheelStates.HubShot))
            .whileFalse(postShootIdles())
        ;
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
