package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
public class Logging {
    private static Logging instance;
    public static Logging getInstance() {
        if (instance == null) {
            instance = new Logging();
        }
        return instance;
    }

    public final DataLog m_log0 = DataLogManager.getLog();
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable evenTable = inst.getTable("EvenLog");

    // Drivetrain logging/telemetry objects
    private final DoubleArrayLogEntry chassisPoseLog = new DoubleArrayLogEntry(m_log0, "DriveState/chassisPose");
    private final StructPublisher<Pose2d> chassisPoseNT = evenTable.getStructTopic("DriveState/chassisPose", Pose2d.struct).publish();

    private final DoubleArrayLogEntry chassisVelocityLog = new DoubleArrayLogEntry(m_log0, "DriveState/chassisVelocity");
    private final StructPublisher<ChassisSpeeds> chassisVelocityNT = evenTable.getStructTopic("DriveState/chassisVelocity", ChassisSpeeds.struct).publish();


    // Drivetrain Pose
    private final DoubleArrayPublisher fieldPub = evenTable.getDoubleArrayTopic("DriveState/Pose/Robot")
        .publish(); // If its not called exactly `robot` elastic wont render it as the robot
    private final StringPublisher fieldTypePub = evenTable.getStringTopic("DriveState/Pose/.type")
        .publish(); // Tells Elastic how to render it
    private final double[] poseArray = new double[3];


    // Who won auton?
    private final StringArrayPublisher autonWinner = evenTable.getStringArrayTopic("AutonWinner").publish();
    private final String[] autonWinnerColorNone = new String[] {"#FF0000", "#0000FF"};
    private final String[] autonWinnerColorError = new String[] {"#00FF00", "#00FF00"};
    private final String[] autonWinnerColorRed = new String[] {"#FF0000", "#000000"};
    private final String[] autonWinnerColorBlue = new String[] {"#000000", "#0000FF"};



    public void logCTREChassis(SwerveDriveState state) {
        chassisPoseLog.append(new double[] {
            state.Pose.getX(), 
            state.Pose.getY(),
            state.Pose.getRotation().getDegrees()
        });

        chassisVelocityLog.append(new double[] {
            state.Speeds.vxMetersPerSecond,
            state.Speeds.vyMetersPerSecond,
            state.Speeds.omegaRadiansPerSecond * 9.549297 // Gets in Rotations Per Minute
        });

        chassisPoseNT.set(state.Pose);
        chassisVelocityNT.set(state.Speeds);

        fieldTypePub.set("Field2d");

        poseArray[0] = state.Pose.getX();
        poseArray[1] = state.Pose.getY();
        poseArray[2] = state.Pose.getRotation().getDegrees();
        fieldPub.set(poseArray);
    }
}
