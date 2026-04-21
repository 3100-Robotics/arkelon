package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import static frc.robot.Constants.CANIds;
import static frc.robot.Constants.IntakeContants;

import java.util.Optional;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.States.Intake.PivotState;
import frc.robot.States.Intake.RollerState;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Intake extends SubsystemBase {
    private CANcoder rawCANCoder = new CANcoder(CANIds.intakeEncoder);
    private TalonFX rawPivotMotor = new TalonFX(CANIds.intakePivot);
    private SparkMax rawRollerMotor = new SparkMax(CANIds.intakeRoller, MotorType.kBrushless);

    private SmartMotorControllerConfig pivotSmcConfig = new SmartMotorControllerConfig(this)
        .withGearing(new MechanismGearing(GearBox.fromStages("4:1", "4:1", "12:48")))

        .withStartingPosition(Degrees.of(0))
        .withClosedLoopController(40,0,0)//IntakeContants.pivotPidReal) //, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        // .withClosedLoopController(IntakeContants.pivotPidSim)

        .withFeedforward(IntakeContants.pivotFeedforwardReal)
        .withSimFeedforward(IntakeContants.pivotFeedforwardSim)

        // .withSoftLimit(Degrees.of(10), Degrees.of(-120))
        .withIdleMode(MotorMode.COAST)
        .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(true)
        // .withClosedLoopRampRate(Seconds.of(0.25))
        // .withOpenLoopRampRate(Seconds.of(0.25))
        .withControlMode(ControlMode.CLOSED_LOOP)


        .withExternalEncoder(rawCANCoder)
        .withExternalEncoderInverted(false)
        .withExternalEncoderGearing(1)
        .withUseExternalFeedbackEncoder(true)

        .withMomentOfInertia(Inches.of(14.724154), Pound.of(7.8858569))
    ;

    private SmartMotorControllerConfig rollerSmcConfig = new SmartMotorControllerConfig(this)
        .withGearing(new MechanismGearing(GearBox.fromStages("36:20")))
        // .withGearing(Math.pow(1.8, -1)) // 36/20
        
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(IntakeContants.rollerPidReal)//, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(IntakeContants.rollerPidSim)//, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        // Feedforward Constants
        .withFeedforward(IntakeContants.rollerFeedforwardReal)
        .withSimFeedforward(IntakeContants.rollerFeedforwardSim)
        // Telemetry name and verbosity level
        .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH)
        // Gearing from the motor rotor to final shaft.
        // Motor properties to prevent over currenting.
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(80))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25))

        .withMomentOfInertia(Inches.of(0.675), Pounds.of(0.9136709))
    ;

    private SmartMotorController pivotMotor = new TalonFXWrapper(rawPivotMotor, DCMotor.getKrakenX60(1), pivotSmcConfig);
    private SmartMotorController rollerMotor = new SparkWrapper(rawRollerMotor, DCMotor.getNEO(1), rollerSmcConfig);

    private Optional<PivotState> pivotState = Optional.empty();
    private Optional<RollerState> rollerState = Optional.empty();

    private Optional<PivotState> lastPivotState = Optional.of(PivotState.Medium);

    public Intake() {
        setDefaultCommand(applyStateCommand());
    }

    public Command togglePivot() {
        return Commands.runOnce(() -> {
            if (pivotState.isEmpty()) {
                pivotState = Optional.of(PivotState.Medium);
            }

            if (pivotState.get() == PivotState.FullDeploy) {
                pivotState = Optional.of(PivotState.Medium);
            } else if (pivotState.get() == PivotState.Medium) {
                pivotState = Optional.of(PivotState.FullDeploy);
            } else if (pivotState.get() == PivotState.HihglyStowed) {
                pivotState = lastPivotState;
            }
            lastPivotState = pivotState;
        });
    }

    public Command setFullStow() {
        return Commands.runOnce(() -> {
            pivotState = Optional.of(PivotState.HihglyStowed);
        });
    }

    public Command leaveFullStow() {
        return Commands.runOnce(() -> {
            pivotState = lastPivotState;
        });
    }

    public Command applyStateCommand() {
        return run(() -> {
            pivotState.ifPresent(state -> {
                pivotMotor.setPosition(state.angle);
            });

            rollerState.ifPresent(state -> {
                if (state.speed.equals(RPM.of(0))) {
                    rollerMotor.setDutyCycle(0);
                } else {
                    rollerMotor.setVelocity(state.speed);
                }
            });
        });
    }

    public Command setPivotState(PivotState pivotState) {
        return Commands.runOnce(() -> this.pivotState = Optional.of(pivotState));
    }

    public Command setRollerState(RollerState rollerState) {
        return Commands.runOnce(() -> this.rollerState = Optional.of(rollerState));
    }

    @Override
    public void periodic() {
        pivotState.ifPresent(state -> SmartDashboard.putString("intakePivot/State", state.toString()));
        SmartDashboard.putNumber("intakePivot/RotorAngle", pivotMotor.getRotorPosition().in(Degrees));
        SmartDashboard.putNumber("intakePivot/MechanisimAngle", pivotMotor.getMechanismPosition().in(Degrees));

        rollerState.ifPresent(state -> SmartDashboard.putString("intakeRoller/State", state.toString()));
        SmartDashboard.putNumber("intakeRoller/RotorSpeed", rollerMotor.getRotorVelocity().in(RPM));
        SmartDashboard.putNumber("intakeRoller/MechanisimSpeed", rollerMotor.getMechanismVelocity().in(RPM));
    }

    @Override
    public void simulationPeriodic() {
        pivotMotor.simIterate();
        rollerMotor.simIterate();
    }
}
