package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripSubsystem;

public class GripCommand extends CommandBase {
    private final GripSubsystem gripSubsystem;

    private final DoubleSupplier gripSupplier;
    
    public GripCommand(GripSubsystem gripSubsystem, DoubleSupplier gripSupplier) {
        this.gripSubsystem = gripSubsystem;
        this.gripSupplier = gripSupplier;

        addRequirements(gripSubsystem);
    }

    @Override
    public void execute() {
        gripSubsystem.move(gripSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        gripSubsystem.move(0);
    }
}
