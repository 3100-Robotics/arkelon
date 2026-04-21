package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import choreo.util.ChoreoAllianceFlipUtil;
import choreo.util.ChoreoAllianceFlipUtil.Flipper;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Constants {
    public static final Distance fieldLength = Inches.of(651.22);
    public static final Distance fieldWidth = Inches.of(317.69);

    public static final Pose2d hubPoseBlue = new Pose2d(4.628518104553223, 4.035704612731934, new Rotation2d());
    public static final Pose2d hubPoseRed = ChoreoAllianceFlipUtil.getMirrorX().flip(hubPoseBlue);

    public static final Pose2d towerPoseBlue = new Pose2d(0.6886858344078064, 4.7454071044921875, new Rotation2d());
    public static final Pose2d towerPoseRed = ChoreoAllianceFlipUtil.getRotateAround().flip(towerPoseBlue);


    public interface MotorConfigs {
        public SparkBaseConfig noBadFilteringNEO = new SparkMaxConfig()
            // TODO: Tune these value for real, currently i just pulled them from the CD thread (thanks Phil)
            // TODO: https://www.chiefdelphi.com/t/psa-rev-spark-default-velocity-filtering-is-still-really-bad-for-flywheels/514567/4
            .apply(new EncoderConfig()
                .quadratureAverageDepth(2)
                .quadratureMeasurementPeriod(10)
            )
        ;
        
    }
        
    public interface CANIds {
        // For all CANId < 20: Reserved for drivetrain

        static int intakePivot = 30;
        static int intakeEncoder = 32;
        static int intakeRoller = 31;

        static int indexerFloor = 40;
        static int indexerCeiling = 41;
        static int indexerKicker = 42;

        static int shooterHoodMotor = 50;
        static int shooterLeftMotor = 51;
        static int shooterRightMotor = 53;
    }

    public interface ShooterConstants {
        static Angle maxHoodAngle = Degrees.of(40);
        static Angle minHoodAngle = Degrees.of(12.667292);
        
        // Shot table
        public class ShotTable {
            public static final List<Pair<Distance, Double>> distanceAngleTable = new ArrayList<>();
            public static final List<Pair<Distance, Double>> distanceSpeedTable = new ArrayList<>();

            static {
                // Distance Angle
                distanceAngleTable.add(Pair.of(Meters.of(1.36), 20.0-2.5));
                distanceAngleTable.add(Pair.of(Meters.of(5.86), 40.0-2.5));

                // Distance Speed
                distanceSpeedTable.add(Pair.of(Meters.of(1.36), 2400.0+(2400*0.05)));
                distanceSpeedTable.add(Pair.of(Meters.of(5.86), 3960.0+(3960*0.05)));
                
            }
        }

        // PID and FF
        static PIDController hoodPidReal = new PIDController(530, 0, 0);
        static PIDController hoodPidSim = new PIDController(5, 0, 0);

        static ArmFeedforward hoodFeedforwardReal = new ArmFeedforward(140, 0, 0);
        static ArmFeedforward hoodFeedforwardSim = new ArmFeedforward(0, 0, 0);


        static PIDController leftPidReal = new PIDController(0.003, 0, 0);
        static PIDController leftPidSim = new PIDController(0.003, 0, 0);

        static SimpleMotorFeedforward leftFeedforwardReal = new SimpleMotorFeedforward(0, 0.137);
        static SimpleMotorFeedforward leftFeedforwardSim = new SimpleMotorFeedforward(0, 0.137);


        static PIDController rightPidReal = new PIDController(0.003, 0, 0);
        static PIDController rightPidSim = new PIDController(0.003, 0, 0);

        static SimpleMotorFeedforward rightFeedforwardReal = new SimpleMotorFeedforward(0,  0.1399);
        static SimpleMotorFeedforward rightFeedforwardSim = new SimpleMotorFeedforward(0,  0.1399);
    }

    public interface IndexerConstants {
        static PIDController floorPidReal = new PIDController(0.01, 0, 0);
        static PIDController floorPidSim = new PIDController(0.05, 0, 0);

        static SimpleMotorFeedforward floorFeedforwardReal = new SimpleMotorFeedforward(0, 0.15);
        static SimpleMotorFeedforward floorFeedforwardSim = new SimpleMotorFeedforward(0, 0.1);


        static PIDController  ceilingPidReal = new PIDController(0.01, 0, 0);
        static PIDController ceilingPidSim = new PIDController(0.05, 0, 0);

        static SimpleMotorFeedforward  ceilingFeedforwardReal = new SimpleMotorFeedforward(0, 0.12);
        static SimpleMotorFeedforward  ceilingFeedforwardSim = new SimpleMotorFeedforward(0, 0.1);

        
        static PIDController kickerPidReal = new PIDController(0.01, 0, 0);
        static PIDController kickerPidSim = new PIDController(0.05, 0, 0);

        static SimpleMotorFeedforward kickerFeedforwardReal = new SimpleMotorFeedforward(0, 0.2);
        static SimpleMotorFeedforward kickerFeedforwardSim = new SimpleMotorFeedforward(0, 0.1);

    }

    public interface IntakeContants {
        // Roller contants
        static PIDController rollerPidReal = new PIDController(10, 0, 0);
        static PIDController rollerPidSim = new PIDController(10, 0, 0);

        static SimpleMotorFeedforward rollerFeedforwardReal = new SimpleMotorFeedforward(0, 0.217, 0);
        static SimpleMotorFeedforward rollerFeedforwardSim = new SimpleMotorFeedforward(0, 0.217, 0);

        // Pivot Constants
        static PIDController pivotPidReal = new PIDController(40, 0, 0);
        static PIDController pivotPidSim = new PIDController(4, 0, 0);

        static ArmFeedforward pivotFeedforwardReal = new ArmFeedforward(0, 0, 0, 0);
        static ArmFeedforward pivotFeedforwardSim = new ArmFeedforward(0, 0, 0, 0);

        static double pivotEncoderDiscontinuityPoint = 0.9;
    }
}
