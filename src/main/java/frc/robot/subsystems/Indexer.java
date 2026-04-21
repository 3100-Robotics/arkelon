package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.States.Indexer.TripleRollerStates;

import static frc.robot.Constants.CANIds;
import static frc.robot.Constants.IndexerConstants;

import java.util.Optional;

import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Indexer extends SubsystemBase {
    private TalonFX rawFloorMotor = new TalonFX(CANIds.indexerFloor);
    private SparkMax rawCeilingMotor = new SparkMax(CANIds.indexerCeiling, MotorType.kBrushless);
    private SparkMax rawKickerMotor = new SparkMax(CANIds.indexerKicker, MotorType.kBrushless);

    private SmartMotorControllerConfig baseRollerConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(60))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25))
        .withMomentOfInertia(KilogramSquareMeters.of(0.001))
    ;

    private SmartMotorControllerConfig floorConfig = baseRollerConfig.clone()
        .withGearing(1.2) // 24/20

        .withMotorInverted(true)

        .withSupplyCurrentLimit(Amps.of(40))
    
        .withClosedLoopController(IndexerConstants.floorPidReal)
        .withSimClosedLoopController(IndexerConstants.floorPidSim)

        .withFeedforward(IndexerConstants.floorFeedforwardReal)
        .withSimFeedforward(IndexerConstants.floorFeedforwardSim)
    ;

    private SmartMotorControllerConfig ceilingConfig = baseRollerConfig.clone()
        .withGearing(1.2) // 24/20

        .withMotorInverted(false)
    
        .withClosedLoopController(IndexerConstants.ceilingPidReal)
        .withSimClosedLoopController(IndexerConstants.ceilingPidSim)

        .withFeedforward(IndexerConstants.ceilingFeedforwardReal)
        .withSimFeedforward(IndexerConstants.ceilingFeedforwardSim)
    ;

    private SmartMotorControllerConfig kickerConfig = baseRollerConfig.clone()
        .withGearing((double)2/(double)3) // 24/36

        .withMotorInverted(false)
    
        .withClosedLoopController(IndexerConstants.kickerPidReal)
        .withSimClosedLoopController(IndexerConstants.kickerPidSim)

        .withFeedforward(IndexerConstants.kickerFeedforwardReal)
        .withSimFeedforward(IndexerConstants.kickerFeedforwardSim)
    ;

    private SmartMotorController floorMotor = new TalonFXWrapper(rawFloorMotor, DCMotor.getKrakenX60(1), floorConfig);
    private SmartMotorController ceilingMotor = new SparkWrapper(rawCeilingMotor, DCMotor.getNEO(1), ceilingConfig);
    private SmartMotorController kickerMotor = new SparkWrapper(rawKickerMotor, DCMotor.getNEO(1), kickerConfig);

    public Optional<TripleRollerStates> tripleRollerStates = Optional.of(TripleRollerStates.Off);

    public Indexer() {
        setDefaultCommand(applyStateCommand());
    }

    public Command setState(TripleRollerStates tripleRollerStates) {
        return Commands.runOnce(() -> this.tripleRollerStates = Optional.of(tripleRollerStates));
    }

    public Command applyStateCommand() {
        return run(() -> {
            if (tripleRollerStates.isEmpty()) {
                return;
            }

            var state = tripleRollerStates.get();

            if (tripleRollerStates.get().compareTo(TripleRollerStates.Off)==0) {
                floorMotor.setDutyCycle(0);
                ceilingMotor.setDutyCycle(0);
                kickerMotor.setDutyCycle(0);
            } else {
                floorMotor.setVelocity(state.floorSpeed);
                ceilingMotor.setVelocity(state.ceilingSpeed);
                kickerMotor.setVelocity(state.kickerSpeed);
            }
        });
    }

    @Override
    public void periodic() {
        tripleRollerStates.ifPresent(state -> SmartDashboard.putString("indexer/State", state.toString()));

        SmartDashboard.putNumber("indexer/floorRotorSpeed", floorMotor.getRotorVelocity().in(RPM));
        SmartDashboard.putNumber("indexer/floorMechanisimSpeed", floorMotor.getMechanismVelocity().in(RPM));

        SmartDashboard.putNumber("indexer/ceilingRotorSpeed", ceilingMotor.getRotorVelocity().in(RPM));
        SmartDashboard.putNumber("indexer/ceilingMechanisimSpeed", ceilingMotor.getMechanismVelocity().in(RPM));

        SmartDashboard.putNumber("indexer/kickerRotorSpeed", kickerMotor.getRotorVelocity().in(RPM));
        SmartDashboard.putNumber("indexer/kickerMechanisimSpeed", kickerMotor.getMechanismVelocity().in(RPM));
    }

    @Override
    public void simulationPeriodic() {
        floorMotor.simIterate();
        ceilingMotor.simIterate();
        kickerMotor.simIterate();
    }
}
