package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Utils.FieldType;

public interface MatchConfig {
    float autonDelay = 5;

    Optional<Alliance> visionColor = Optional.empty();
    Optional<FieldType> fieldType = Optional.empty();
}
