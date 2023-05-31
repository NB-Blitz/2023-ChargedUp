package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.ChargingBalance;
import frc.robot.commands.AutoCommands.DriveTime;
import frc.robot.commands.AutoCommands.ManipulatorPreset;
import frc.robot.commands.AutoCommands.MoveGrip;
import frc.robot.commands.AutoCommands.RotateToAngle;
import frc.robot.subsystems.GripSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Autonomous extends SequentialCommandGroup {

    public Autonomous(String choice,
                      SwerveSubsystem swerveSubsystem,
                      ManipulatorSubsystem manipulatorSubsystem,
                      GripSubsystem gripSubsystem) {
        addRequirements(swerveSubsystem, manipulatorSubsystem, gripSubsystem);
        
        switch(choice) {
            case "leaveC":
                // Drive forward out of community
                addCommands(new DriveTime("Leave Community", 0.5, 0, 0, 1.9, swerveSubsystem));
                break;
            case "scoreHighR":
                // Place cube on top shelf, strafe right, leave community
                addCommands(
                    new ManipulatorPreset("high", manipulatorSubsystem),
                    new DriveTime("Approach Grid",-0.2, 0, 0, 1.5, swerveSubsystem),
                    //new RotateToAngle(0, swerveSubsystem),
                    new MoveGrip(-1, 2, gripSubsystem),
                    new DriveTime("Back Away", 0.3, 0, 0, 1, swerveSubsystem),
                    new ManipulatorPreset("home", manipulatorSubsystem),
                    new DriveTime("Strafe", 0, -0.3, 0, 0.5, swerveSubsystem), //on left +y on right -y looking to the drivers
                    new DriveTime("Leave Community", 0.6, 0, 0, 1.7, swerveSubsystem),
                    new RotateToAngle(180, swerveSubsystem)
                );
                break;
            case "scoreHighL":
                // Place cube on top shelf, strafe left, leave community
                addCommands(
                    new ManipulatorPreset("high", manipulatorSubsystem),
                    new DriveTime("Approach Grid",-0.2, 0, 0, 1.5, swerveSubsystem),
                    //new RotateToAngle(0, swerveSubsystem),
                    new MoveGrip(-1, 2, gripSubsystem),
                    new DriveTime("Back Away", 0.3, 0, 0, 1, swerveSubsystem),
                    new ManipulatorPreset("home", manipulatorSubsystem),
                    new DriveTime("Strafe", 0, 0.3, 0, 0.5, swerveSubsystem), //on left +y on right -y looking to the drivers
                    new DriveTime("Leave Community", 0.6, 0, 0, 1.7, swerveSubsystem),
                    new RotateToAngle(180, swerveSubsystem)
                );
                break;
            case "scoreHighMBalance":
                // Place cube on top shelf, engage on the charging station
                addCommands(
                    new ManipulatorPreset("high", manipulatorSubsystem),
                    new DriveTime("Approach Grid", -0.3, 0, 0, 1, swerveSubsystem),
                    //new RotateToAngle(0, swerveSubsystem),
                    new MoveGrip(-1, 2, gripSubsystem),
                    new DriveTime("Back Away", 0.3, 0, 0, 1, swerveSubsystem),
                    new ManipulatorPreset("home", manipulatorSubsystem),
                    new DriveTime("Climb Charging Station", 0.6, 0, 0, 2, swerveSubsystem),
                    new ChargingBalance(swerveSubsystem)
                );
                break;
            case "scoreHighM":
                addCommands(
                    new ManipulatorPreset("high", manipulatorSubsystem),
                    new DriveTime("Approach Grid",-0.2, 0, 0, 1.5, swerveSubsystem),
                    //new RotateToAngle(0, swerveSubsystem),
                    new MoveGrip(-1, 2, gripSubsystem)
                    //new DriveTime("Back Away", 0.2, 0, 0, 1.3, swerveSubsystem),
                    //new ManipulatorPreset("home", manipulatorSubsystem)
                );
            case "scoreMiddleM":
                addCommands(
                    new ManipulatorPreset("middle", manipulatorSubsystem),
                    new DriveTime("Approach Grid",-0.2, 0, 0, 1.5, swerveSubsystem),
                    //new RotateToAngle(0, swerveSubsystem),
                    new MoveGrip(-1, 2, gripSubsystem)
                    //new DriveTime("Back Away", 0.2, 0, 0, 1.3, swerveSubsystem),
                    //new ManipulatorPreset("home", manipulatorSubsystem)
                );
                break;
            case "forgotPreload":
                addCommands(
                    new DriveTime("Score Hybrid", -0.3, 0, 0, 0.8, swerveSubsystem),
                    new DriveTime("Leave Community", 0.5, 0, 0, 2, swerveSubsystem)
                );
                break;
                case "scoreMidL":
                // Place cube on middle shelf, strafe left, leave community
                addCommands(
                    new ManipulatorPreset("middle", manipulatorSubsystem),
                    new DriveTime("Approach Grid",-0.2, 0, 0, 1.5, swerveSubsystem),
                    //new RotateToAngle(0, swerveSubsystem),
                    new MoveGrip(-1, 2, gripSubsystem),
                    new DriveTime("Back Away", 0.3, 0, 0, 1, swerveSubsystem),
                    new ManipulatorPreset("home", manipulatorSubsystem),
                    new DriveTime("Strafe", 0, 0.3, 0, 0.5, swerveSubsystem), //on left +y on right -y looking to the drivers
                    new DriveTime("Leave Community", 0.6, 0, 0, 1.7, swerveSubsystem),
                    new RotateToAngle(180, swerveSubsystem)
                );
                break;
                case "scoreMidR":
                // Place cube on middle shelf, strafe right, leave community
                addCommands(
                    new ManipulatorPreset("middle", manipulatorSubsystem),
                    new DriveTime("Approach Grid",-0.2, 0, 0, 1.5, swerveSubsystem),
                    //new RotateToAngle(0, swerveSubsystem),
                    new MoveGrip(-1, 2, gripSubsystem),
                    new DriveTime("Back Away", 0.3, 0, 0, 1, swerveSubsystem),
                    new ManipulatorPreset("home", manipulatorSubsystem),
                    new DriveTime("Strafe", 0, -0.3, 0, 0.5, swerveSubsystem), //on left +y on right -y looking to the drivers
                    new DriveTime("Leave Community", 0.6, 0, 0, 1.7, swerveSubsystem),
                    new RotateToAngle(180, swerveSubsystem)
                );
                break;
            default:
                // Do nothing
        }
    }
}
