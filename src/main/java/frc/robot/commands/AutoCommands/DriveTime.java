package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveTime extends Command {
    private double x;
    private double y;
    private double rotation;
    private final double time;
    private final Timer timerDrive;
    private SwerveSubsystem swerveSubsystem;
    private ShuffleboardTab tab;

    public DriveTime(String title, double x, double y, double rotation, double time, SwerveSubsystem swerveSubsystem) {
        this.x = x;
        this.y = y;
        this.rotation = rotation;
        this.time = time;
        this.timerDrive = new Timer();
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);

        tab = Shuffleboard.getTab("Autonomous");
        tab.addDouble("Timer " + title, () -> timerDrive.get());
    }

    @Override
    public void initialize() {
        timerDrive.restart();
    }

    @Override
    public void execute() {
        swerveSubsystem.drive(
            new Translation2d(x, y).times(Constants.Swerve.maxSpeed),
            rotation * Constants.Swerve.maxAngularVelocity,
            true, true
        );
    }

    @Override
    public boolean isFinished() {
        return timerDrive.get() >= time;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new Translation2d(0, 0), 0, true, true);
        timerDrive.stop();
    }
}
