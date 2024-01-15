package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateToAngle extends Command {
    private int targetAngle;
    private SwerveSubsystem swerveSubsystem;

    private final int ANGLE_ERROR = 2;
    
    public RotateToAngle(int targetAngle, SwerveSubsystem swerveSubsystem) {
        this.targetAngle = targetAngle;
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        double currentAngle = swerveSubsystem.getYaw().getDegrees();
        int direction;
        if (currentAngle < targetAngle - ANGLE_ERROR) {
            if (targetAngle - currentAngle > 180) {
                direction = 1;
            } else {
                direction = -1;
            }
        } else if (currentAngle > targetAngle + ANGLE_ERROR) {
            if (currentAngle - targetAngle > 180) {
                direction = -1;
            } else {
                direction = 1;
            }
        } else {
            direction = 0;
        }
        swerveSubsystem.drive(
            new Translation2d(0, 0),
            0.2 * direction * Constants.Swerve.maxAngularVelocity,
            true, true
        );
    }

    @Override
    public boolean isFinished() {
        double currentAngle = swerveSubsystem.getYaw().getDegrees();
        return currentAngle >= targetAngle - ANGLE_ERROR && currentAngle <= targetAngle + ANGLE_ERROR;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new Translation2d(0, 0), 0, true, true);
    }
}
