package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private SwerveSubsystem s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier slowModeSup;
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(SwerveSubsystem s_Swerve,
                        DoubleSupplier translationSup,
                        DoubleSupplier strafeSup,
                        DoubleSupplier rotationSup,
                        BooleanSupplier slowModeSup,
                        BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.slowModeSup = slowModeSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.xyDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.xyDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.twistDeadband);

        if (slowModeSup.getAsBoolean()) {
            translationVal *= 0.4;
            strafeVal *= 0.4;
            rotationVal *= 0.3;
        } else {
            translationVal *= 0.8;
            strafeVal *= 0.8;
            rotationVal *= 0.6;
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
            rotationVal * Constants.Swerve.maxAngularVelocity,
            !robotCentricSup.getAsBoolean(),
            true
        );
    }
}