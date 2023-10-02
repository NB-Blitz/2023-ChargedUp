// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static SwerveConfigs swerveConfigs = new SwerveConfigs();
    //swerveConfigs = new SwerveConfigs();
   // private Command m_autonomousCommand;

   // private RobotContainer m_robotContainer;


    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();
    private final GripSubsystem gripSubsystem = new GripSubsystem();

    private final Joystick joystick = new Joystick(0);
    private final XboxController controller = new XboxController(1);

    private final boolean robotOriented = false;

    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        swerveConfigs = new SwerveConfigs();
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        //m_robotContainer = new RobotContainer();
        //CameraServer.startAutomaticCapture();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        //CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
       // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // Schedule the autonomous command
       // if (m_autonomousCommand != null) {
         //   m_autonomousCommand.schedule();
        //}
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
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

        //configureButtonBindings();

        
        
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

        processDrive(swerveSubsystem, joystick.getY(), joystick.getX(), joystick.getZ(), joystick.getRawButton(12), robotOriented);

        /* swerveSubsystem.
                swerveSubsystem,
                () -> joystick.getY(),
                () -> joystick.getX()
                () -> joystick.getTwist(),
                () -> joystick.getRawButton(12),
                () -> robotOriented
            )
        ); */
        /*
        manipulatorSubsystem.setDefaultCommand(new ManipulatorCommand(
            manipulatorSubsystem,
            () -> -MathUtil.applyDeadband(controller.getLeftY(), 0.2),
            () -> translateBumpers(controller.getLeftBumper(), controller.getRightBumper()),
            () -> -MathUtil.applyDeadband(controller.getRawAxis(5), 0.2)
        ));

        gripSubsystem.setDefaultCommand(new GripCommand(
            gripSubsystem,
            () -> translateTriggers(controller.getLeftTriggerAxis(), controller.getRightTriggerAxis())));
            */
    }
    

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

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

    private static void processDrive(SwerveSubsystem s_Swerve, Double Y, Double X, Double Z, Boolean slow, Boolean roboticCentric){
        double translationVal = MathUtil.applyDeadband(Y, Constants.xyDeadband);
        double strafeVal = MathUtil.applyDeadband(X, Constants.xyDeadband);
        double rotationVal = MathUtil.applyDeadband(Z, Constants.twistDeadband);

        if (slow) {
            translationVal *= 0.4;
            strafeVal *= 0.4;
            rotationVal *= 0.3;
        } else {
            translationVal *= 0.8;
            strafeVal *= 0.8;
            rotationVal *= 0.6;
        }

        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
            rotationVal * Constants.Swerve.maxAngularVelocity,
            !roboticCentric,
            true
        );
    }

    
}
