package frc.robot.commands;

import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ComposeableAuto {
    public AutoRoutine routine;
    public Supplier<Command> mainCommandGenerator;

    public ComposeableAuto(String name, AutoFactory autoFactory, Supplier<Command> mainCommandGenerator) {
        this.routine = autoFactory.newRoutine(name);
        this.routine.active().onTrue(null);
    }
}
