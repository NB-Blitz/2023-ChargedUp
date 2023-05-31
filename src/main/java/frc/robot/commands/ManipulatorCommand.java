package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;
import java.util.function.DoubleSupplier;

/*
 * wrist angle facing down = -90 degrees when shoulder down DEFAULT
 * wrist angle facing forward = 0 degrees when shoulder down
 * shoulder angle facing down = 0 DEFAULT
 * change wrist angle by shoulder angle * -1
 */

public class ManipulatorCommand extends CommandBase {
    private final ManipulatorSubsystem manipulatorStateSubsystem;
    private final DoubleSupplier shoulderSupplier;
    private final DoubleSupplier telescopeSupplier;
    private final DoubleSupplier wristSupplier;

    public ManipulatorCommand(ManipulatorSubsystem manipulatorStateSubsystem,
                              DoubleSupplier shoulderSupplier,
                              DoubleSupplier telescopeSupplier,
                              DoubleSupplier wristSupplier) {
        this.manipulatorStateSubsystem = manipulatorStateSubsystem;
        this.shoulderSupplier = shoulderSupplier;
        this.telescopeSupplier = telescopeSupplier;
        this.wristSupplier = wristSupplier;
        
        addRequirements(manipulatorStateSubsystem);
    }

    @Override
    public void execute() {
        manipulatorStateSubsystem.move(
            shoulderSupplier.getAsDouble(),
            telescopeSupplier.getAsDouble(),
            wristSupplier.getAsDouble()
        );
    }

    @Override
    public void end(boolean interrupted) {
        manipulatorStateSubsystem.setHome();
    }
}
