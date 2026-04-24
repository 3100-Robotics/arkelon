package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utils.Acceleration;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.CANIds;
import static frc.robot.Constants.ShooterConstants;
import static frc.robot.States.Shooter.HoodState;
import static frc.robot.States.Shooter.FlywheelStates;

import java.util.Optional;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
    private TalonFX rawHoodMotor = new TalonFX(CANIds.shooterHoodMotor);
    private SparkMax rawLeftMotor = new SparkMax(CANIds.shooterLeftMotor, MotorType.kBrushless);
    private SparkMax rawRightMotor = new SparkMax(CANIds.shooterRightMotor, MotorType.kBrushless);

    private SmartMotorControllerConfig hoodConfig = new SmartMotorControllerConfig(this)
        .withStatorCurrentLimit(Amps.of(50))
        .withSupplyCurrentLimit(Amps.of(40))

        .withIdleMode(MotorMode.COAST)

        .withGearing(new MechanismGearing(GearBox.fromStages("48:12", "182:10")))
        .withControlMode(ControlMode.CLOSED_LOOP)

        .withClosedLoopController(ShooterConstants.hoodPidReal)
        .withSimClosedLoopController(ShooterConstants.hoodPidSim)
        .withFeedforward(ShooterConstants.hoodFeedforwardReal)
        .withSimFeedforward(ShooterConstants.hoodFeedforwardSim)

        .withMotorInverted(true)
        .withStartingPosition(Degrees.of(12.667292))
        .withMomentOfInertia(KilogramSquareMeters.of(0.0190245794))
        
        .withTelemetry("shooterHood", TelemetryVerbosity.HIGH)
    ;

    private SmartMotorControllerConfig baseFlywheelConfig = new SmartMotorControllerConfig(this)
        .withVendorConfig(Constants.MotorConfigs.noBadFilteringNEO)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(60))

        .withMomentOfInertia(Inches.of(1.9825395), Pounds.of(0.9))
    ;

    private SmartMotorControllerConfig leftFlywheelConfig = baseFlywheelConfig.clone()
        .withClosedLoopController(ShooterConstants.leftPidReal)
        .withSimClosedLoopController(ShooterConstants.leftPidSim)
        .withFeedforward(ShooterConstants.leftFeedforwardReal)
        .withSimFeedforward(ShooterConstants.leftFeedforwardSim)
        .withMotorInverted(true)
    ;
    private SmartMotorControllerConfig rightFlywheelConfig = baseFlywheelConfig.clone()
        .withClosedLoopController(ShooterConstants.rightPidReal)
        .withSimClosedLoopController(ShooterConstants.rightPidSim)
        .withFeedforward(ShooterConstants.rightFeedforwardReal)
        .withSimFeedforward(ShooterConstants.rightFeedforwardSim)
        .withMotorInverted(false)
    ;


    private SmartMotorController hoodMotor = new TalonFXWrapper(rawHoodMotor, DCMotor.getKrakenX60(1), hoodConfig);

    private SmartMotorController leftMotor = new SparkWrapper(rawLeftMotor, DCMotor.getNEO(1), leftFlywheelConfig);
    private SmartMotorController rightMotor = new SparkWrapper(rawRightMotor, DCMotor.getNEO(1), rightFlywheelConfig);

    public Acceleration leftAcceleration = new Acceleration(0);
    public Acceleration rightAcceleration = new Acceleration(0);
    public Trigger flywheelsAtAccel = new Trigger(() -> {
        return leftAcceleration.getAccel() < 950 && rightAcceleration.getAccel() < 950;
    }).debounce(0.05);

    private Optional<HoodState> hoodState = Optional.of(HoodState.Frozen);
    private Optional<FlywheelStates> flywheelStates = Optional.of(FlywheelStates.Frozen);

    public Supplier<Angle> hoodAngleSupplier;
    public Supplier<AngularVelocity> flywheelSpeedSupplier;

    public Shooter(Supplier<Angle> hoodAngleSupplier, Supplier<AngularVelocity> flywheelSpeedSupplier) {
        this.hoodAngleSupplier = hoodAngleSupplier;
        this.flywheelSpeedSupplier = flywheelSpeedSupplier;
        Robot.getInstance().addPeriodic(this::updateAcceleration, 0.02);
        setDefaultCommand(applyStateCommand());
    }

    public Command applyStateCommand() {
        return run(() -> {
            hoodState.ifPresent(state -> {
                if (state==HoodState.Frozen) {
                    hoodMotor.setDutyCycle(0);
                } else if (state==HoodState.Varying) {
                    hoodMotor.setPosition(hoodAngleSupplier.get());
                } else if (state==HoodState.Pass) {
                    hoodMotor.setPosition(hoodAngleSupplier.get());
                } else if (state.angle.isPresent()) {
                    hoodMotor.setPosition(state.angle.get());
                }
            });

            flywheelStates.ifPresent(state -> {
                if (state==FlywheelStates.Frozen) {
                    leftMotor.setDutyCycle(0);
                    rightMotor.setDutyCycle(0);
                } else if (state==FlywheelStates.Varying) {
                    leftMotor.setVelocity(flywheelSpeedSupplier.get());
                    rightMotor.setVelocity(flywheelSpeedSupplier.get());
                } else if (state==FlywheelStates.Pass) {
                    leftMotor.setVelocity(flywheelSpeedSupplier.get());
                    rightMotor.setVelocity(flywheelSpeedSupplier.get());
                } else if (state.speed.isPresent()) {
                    leftMotor.setVelocity(state.speed.get());
                    rightMotor.setVelocity(state.speed.get());
                }
            });

        });
    }

    public Command setHoodState(HoodState hoodState) {
        return Commands.runOnce(() -> this.hoodState = Optional.of(hoodState));
    }

    public Command setFlywheelState(FlywheelStates flywheelStates) {
        return Commands.runOnce(() -> this.flywheelStates = Optional.of(flywheelStates));
    }

    public void updateAcceleration() {
        leftAcceleration.update(leftMotor.getMechanismVelocity().in(RPM));
        rightAcceleration.update(rightMotor.getMechanismVelocity().in(RPM));
    }

    @Override
    public void periodic() {
        hoodState.ifPresent(state -> SmartDashboard.putString("shooterHood/State", state.toString()));
        flywheelStates.ifPresent(state -> SmartDashboard.putString("shooterFlywheels/State", state.toString()));

        SmartDashboard.putNumber("shooterHood/RotorAngle", hoodMotor.getRotorPosition().in(Degrees));
        SmartDashboard.putNumber("shooterHood/MechanisimAngle", hoodMotor.getMechanismPosition().in(Degrees));

        SmartDashboard.putNumber("shooterFlywheels/leftMechanisimSpeed", leftMotor.getMechanismVelocity().in(RPM));

        SmartDashboard.putNumber("shooterFlywheels/rightMechanisimSpeed", rightMotor.getMechanismVelocity().in(RPM));

        SmartDashboard.putNumber("flywheelsAtAccel", flywheelsAtAccel.getAsBoolean() ? 3000 : 1000);
        SmartDashboard.putNumber("flywheelAccel", leftAcceleration.getAccel());
    }

    @Override
    public void simulationPeriodic() {
        hoodMotor.simIterate();
        leftMotor.simIterate();
        rightMotor.simIterate();
    }

    public static Angle calculateHoodAngle(Distance shooterDistToHub) {
        // var shooterDistToHub = 
        //     Locator.getInstance().getDistanceToHub() // Robot center distance from hub
        //     .plus(Inches.of(5.4330709)) // Center to fuel exit
        // ;
        var angleTable = Constants.ShooterConstants.ShotTable.distanceAngleTable;

        double angleOut = 15; // Default
        if (shooterDistToHub.lt(angleTable.get(0).getFirst())) {
            angleOut = Constants.ShooterConstants.minHoodAngle.in(Degrees);
        }

        if (shooterDistToHub.gt(angleTable.get(angleTable.size()-1).getFirst())) {
            angleOut = Constants.ShooterConstants.maxHoodAngle.in(Degrees);
        }

        for (int i = 0; i < Constants.ShooterConstants.ShotTable.distanceAngleTable.size()-1; i++) {
            if (shooterDistToHub.gte(angleTable.get(i).getFirst()) && 
                shooterDistToHub.lte(angleTable.get(i+1).getFirst())
            ) {
                var x1 = angleTable.get(i).getFirst().in(Feet);
                var x2 = angleTable.get(i+1).getFirst().in(Feet);
                var y1 = angleTable.get(i).getSecond();
                var y2 = angleTable.get(i+1).getSecond();

                var m = (y2 - y1) / (x2 - x1);
                var b = y1 - m * x1;
                angleOut = m * shooterDistToHub.in(Feet) + b;
            }
        }

        return Degrees.of(angleOut);
    }

    public static AngularVelocity calculateFlywheelSpeed(Distance shooterDistToHub) {
        var angleTable = Constants.ShooterConstants.ShotTable.distanceAngleTable;
        var speedTable = Constants.ShooterConstants.ShotTable.distanceSpeedTable;

        
        double speedOut = 4000;
        if (shooterDistToHub.lt(speedTable.get(0).getFirst())) {
            speedOut = 31;
        }

        if (shooterDistToHub.gt(speedTable.get(angleTable.size()-1).getFirst())) {
            speedOut = 3100;
        }

        for (int i = 0; i < Constants.ShooterConstants.ShotTable.distanceAngleTable.size()-1; i++) {
            if (shooterDistToHub.gte(speedTable.get(i).getFirst()) && 
                shooterDistToHub.lte(speedTable.get(i+1).getFirst())
            ) {
                var x1 = speedTable.get(i).getFirst().in(Feet);
                var x2 = speedTable.get(i+1).getFirst().in(Feet);
                var y1 = speedTable.get(i).getSecond();
                var y2 = speedTable.get(i+1).getSecond();

                var m = (y2 - y1) / (x2 - x1);
                var b = y1 - m * x1;
                speedOut = m * shooterDistToHub.in(Feet) + b;
            }
        }

        return RPM.of(speedOut);
    }
}
