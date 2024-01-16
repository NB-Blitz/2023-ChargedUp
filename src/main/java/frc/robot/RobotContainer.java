// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Autonomous;
import frc.robot.commands.GripCommand;
import frc.robot.commands.ManipulatorCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.GripSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.simulation.FieldSim;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();
    private final GripSubsystem gripSubsystem = new GripSubsystem();
    private final FieldSim m_fieldSim = new FieldSim(swerveSubsystem);

    private final Joystick joystick = new Joystick(0);
    private final XboxController controller = new XboxController(1);

    private final boolean robotOriented = false;

    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving

        autoChooser.setDefaultOption("Do Nothing", "nothing");
        autoChooser.addOption("Start L/R: Leave Community", "leaveC");
        autoChooser.addOption("Start Left: Score High", "scoreHighL");
        autoChooser.addOption("Start Right: Score High", "scoreHighR");
        autoChooser.addOption("Start Middle: Score High, Balance", "scoreHighMBalance");
        autoChooser.addOption("Start Middle: Score High", "scoreHighM");
        autoChooser.addOption("Start Middle: Score Middle", "scoreMiddleM");
        autoChooser.addOption("Forgot To Preload 😭", "forgotPreload");
        autoChooser.addOption("Start Right: Score Middle", "scoreMidR");
        autoChooser.addOption("Start Left: Score Middle", "scoreMidL");
        Shuffleboard.getTab("Autonomous").add(autoChooser);

        configureButtonBindings();

        swerveSubsystem.setDefaultCommand(
            new TeleopSwerve(
                swerveSubsystem,
                () -> joystick.getY(),
                () -> joystick.getX(),
                () -> joystick.getTwist(),
                () -> joystick.getRawButton(12),
                () -> robotOriented
            )
        );
        
        manipulatorSubsystem.setDefaultCommand(new ManipulatorCommand(
            manipulatorSubsystem,
            () -> -MathUtil.applyDeadband(controller.getLeftY(), 0.2),
            () -> translateBumpers(controller.getLeftBumper(), controller.getRightBumper()),
            () -> -MathUtil.applyDeadband(controller.getRawAxis(5), 0.2)
        ));

        gripSubsystem.setDefaultCommand(new GripCommand(
            gripSubsystem,
            () -> translateTriggers(controller.getLeftTriggerAxis(), controller.getRightTriggerAxis())));
    
        m_fieldSim.initSim();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Button 11 on the joystick zeros the gyroscope controller
        new JoystickButton(joystick, 11)
            .onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));
        
        // A sets manipulator to home position
        new JoystickButton(controller, 1)
            .onTrue(new InstantCommand(() -> manipulatorSubsystem.setHome()));

        // X sets manipulator to floor position
        new JoystickButton(controller, 3)
            .onTrue(new InstantCommand(() -> manipulatorSubsystem.setFloor()));

        // Y sets manipulator to lower scoring position
        new JoystickButton(controller, 4)
            .onTrue(new InstantCommand(() -> manipulatorSubsystem.setTwo()));

        // B sets manipulator to higher scoring position
        new JoystickButton(controller, 2)
            .onTrue(new InstantCommand(() -> manipulatorSubsystem.setThree()));

        // Right joystick button sets manipulator to player station position
        new JoystickButton(controller, 8)
            .onTrue(new InstantCommand(() -> manipulatorSubsystem.setPlayerShelf()));

        new JoystickButton(controller, 7)
            .onTrue(new InstantCommand(() -> manipulatorSubsystem.killWrist(true)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        swerveSubsystem.zeroGyro();
        return new Autonomous(autoChooser.getSelected(), swerveSubsystem, manipulatorSubsystem, gripSubsystem);
    }

    private static double translateTriggers(double left, double right) {
        return -left + right;
    }

    private static double translateBumpers(boolean left, boolean right) {
        double result = 0.0;
        if (left) result--;
        if (right) result++;
        return result;
    }

    public void periodic() {
        //checkDSUpdate();
    
        m_fieldSim.periodic();
        //m_mechanismSimulator.periodic();
    
        // TODO: Fix, This crashes code
        // if(m_timeOfFlight.sensorDetected()){
        //   m_CurrentPitchIntakeCommand = m_LowPitchIntakeCommand;
        // }
        // else{
        //   m_CurrentPitchIntakeCommand = m_HighPitchIntakeCommand;
        // }
      }
}
