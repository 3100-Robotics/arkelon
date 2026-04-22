package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import java.util.Optional;
import java.util.function.Supplier;

import choreo.util.ChoreoAllianceFlipUtil;
import choreo.util.ChoreoAllianceFlipUtil.Flipper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

public class Locator {
    public static Locator instance;
    public Drivetrain drivetrain;

    public boolean hasAppliedAlliance = false;
    public Optional<Alliance> alliance = Optional.empty();
    public Rotation2d hubPointFlipAngle = Rotation2d.kZero;

    public Pose2d hubPose = Constants.hubPoseBlue;
    public Pose2d towerPose = Constants.towerPoseBlue;

    private final Field2d field = new Field2d();

    public Locator(Supplier<Pose2d> getRobotPose, Drivetrain drivetrain) {
        if (instance == null) {
            instance = this;
        }
        // this.getRobotPose = getRobotPose;
        this.drivetrain = drivetrain;
    }

    public void setupPeriodics() {
        Robot.getInstance().addPeriodic(this::periodicAllianceApplied, 0.1);
        Robot.getInstance().addPeriodic(this::periodicSmartDashboard, 0.1);
    }

    public static Locator getInstance() {
        return instance;
    }

    public Pose2d getRobotPose() {
        return drivetrain.getPos();
    }

    public Pose2d getExtensionPose() {
        var thub = new Rotation2d(Math.PI+Math.atan2(hubPose.getY()-getRobotPose().getY(), hubPose.getX()-getRobotPose().getX()));
        return new Pose2d(
            2.1*Math.cos(thub.getRadians())+hubPose.getX(),
            2.1*Math.sin(thub.getRadians())+hubPose.getY(),
            thub.rotateBy(Rotation2d.k180deg)
        );
    }

    public Distance getDistanceToHub() {
        double distance = Math.sqrt(
            Math.pow(getRobotPose().getMeasureX().minus(hubPose.getMeasureX()).in(Inches), 2) + 
            Math.pow(getRobotPose().getMeasureY().minus(hubPose.getMeasureY()).in(Inches), 2)
        );
        return Inches.of(distance);
    }


    // Periodics

    // Smartdashboard values every 0.1 seconds
    public void periodicSmartDashboard() {
        SmartDashboard.putNumber("shooterTargetSpeed2", RobotContainer.getInstance().shooter.flywheelSpeedSupplier.get().in(RPM));
        SmartDashboard.putData("best field ever", field);
        SmartDashboard.putBoolean("recvAllianceColor", hasAppliedAlliance);
        // SmartDashboard.putNumber("robotDistanceToHubFeet", distanceToHub.get().in(Feet));
        field.setRobotPose(getRobotPose());
    }

    // 0.1 second
    public void periodicAllianceApplied() {
        if (!hasAppliedAlliance || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                alliance = Optional.of(allianceColor);
                hasAppliedAlliance = true;
            });
        
            alliance.ifPresent(
                allianceColor -> hubPose = allianceColor == Alliance.Red 
                    ? Constants.hubPoseRed : 
                    Constants.hubPoseBlue
            );

            alliance.ifPresent(
                allianceColor -> towerPose = allianceColor == Alliance.Red 
                    ? Constants.towerPoseRed : 
                    Constants.towerPoseBlue
            );

            alliance.ifPresent(
                allianceColor -> hubPointFlipAngle = allianceColor == Alliance.Red 
                    ? Rotation2d.kZero: 
                    Rotation2d.k180deg
            );

            SmartDashboard.putString("lookatme", alliance.toString());
            field.getObject("hub22").setPose(hubPose);
        }
    }
}