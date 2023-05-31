package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class ChargingBalance extends CommandBase {
    private static final double INVERTED_DEADBAND = 11.0;
    private static final double NON_INVERTED_DEADBAND = 12.0;
    private final double MAX_SPEED = 0.15;
    private final double STRAFE = 0.0;
    private final double ROTATION = 0.0;
    private final boolean IS_FIELD_RELATIVE = true;
    private final boolean IS_OPEN_LOOP = true;

    private SwerveSubsystem swerveSubsystem;

    private boolean inverted;

    /**
     * <h3>AutoBalanceCommand</h3>
     * 
     * Balances the robot on the Charging Station
     * 
     * @param swerveSubsystem Drive subsystem
     */
    public ChargingBalance(SwerveSubsystem swerveSubsystem, boolean inverted) {
        this.swerveSubsystem = swerveSubsystem;
        this.inverted = inverted;
        addRequirements(swerveSubsystem);
    }
    public ChargingBalance(SwerveSubsystem swerveDrive) {
        this(swerveDrive, false);
    }

    @Override
    public void execute() {
        // Get pitch in degrees from swerve drive subsystem
        double robotPitchInDegrees = swerveSubsystem.getPitch();
        robotPitchInDegrees = MathUtil.applyDeadband(robotPitchInDegrees, 
            inverted ? INVERTED_DEADBAND : NON_INVERTED_DEADBAND, // TODO TUNE deadband to balance
            15.0); 

        // Gets percentage of max speed to set swerve drive to
        double tempSpeed = MathUtil.clamp(swerveSubsystem.getAutoPitchController().calculate(robotPitchInDegrees, 0.0), -1.0, 1.0);
        
        // Sets drive to throttle
        double throttle = ((inverted) ? -1.0 : 1.0) * tempSpeed * MAX_SPEED * Constants.Swerve.maxSpeed;
        swerveSubsystem.drive(new Translation2d(throttle, STRAFE), ROTATION, IS_FIELD_RELATIVE, IS_OPEN_LOOP);
    }

    @Override
    public boolean isFinished() {
        return false; // Shouldn't end until switched to teleop
    }
}
