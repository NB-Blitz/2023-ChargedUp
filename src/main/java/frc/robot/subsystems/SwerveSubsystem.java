package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.REVPhysicsSim;
import com.ctre.phoenix.unmanaged.Unmanaged;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    public static final double kTrackWidth = Units.inchesToMeters(18.5);// 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20.5);// 0.7;
    // Distance between front and back wheels on robot
  
    public static final Translation2d[] kModuleTranslations = {
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    };

    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;

    public AHRS gyro;
    private double gyroOffset;
    private double m_simYaw;
    private double m_simYawRawHeading;

    private PIDController autoPitchController;

    private ShuffleboardTab tab;

    public SwerveSubsystem() {
        gyro = new AHRS(SPI.Port.kMXP);
        gyroOffset = 0.0;
        autoPitchController = new PIDController(1, 0, 0);
        tab = Shuffleboard.getTab("Drivetrain");

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        for(SwerveModule mod : mSwerveMods){
            tab.addDouble("Mod " + mod.moduleNumber + " Cancoder", () -> mod.getCanCoder().getDegrees());
            tab.addDouble("Mod " + mod.moduleNumber + " Integrated", () -> mod.getPosition().angle.getDegrees());
            tab.addDouble("Mod " + mod.moduleNumber + " Velocity", () -> mod.getState().speedMetersPerSecond);
        }

        tab.addDouble("Gyro Yaw", () -> getYaw().getDegrees());
        tab.addDouble("Gyro Pitch", () -> getPitch());

        /* 
         * By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.zeroYaw();
        gyroOffset = getRawYaw();
        if (RobotBase.isSimulation()) {
            m_simYaw = 0;
            m_simYawRawHeading = 0;
        }
    }

    public Rotation2d getYaw() {
        double adjusted = getRawYaw() - gyroOffset;
        if (adjusted < 0) {
            adjusted = 360 + adjusted;
        }
        if (RobotBase.isReal())
            return Rotation2d.fromDegrees(adjusted);
        else
            return Rotation2d.fromDegrees(getRawYaw());
    }

    public double getRawYaw() {
        if (RobotBase.isReal())
            return Constants.Swerve.invertGyro ? 360 - gyro.getFusedHeading() : gyro.getFusedHeading();
        else
            return Constants.Swerve.invertGyro ? (360 - m_simYawRawHeading) : m_simYawRawHeading;
    }

    public double getPitch() {
        if (RobotBase.isReal())
            return Constants.Swerve.invertGyro ? -1 * gyro.getPitch() : gyro.getPitch();
        else
            return 0;
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());
    }

    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();
        ChassisSpeeds chassisSpeed = Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
        m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;
    
        m_simYawRawHeading = (-Units.radiansToDegrees(m_simYaw));
    }

    public PIDController getAutoPitchController() {
        return autoPitchController;
    }

    public SwerveModule getSwerveModule(int moduleNumber) {
        return mSwerveMods[moduleNumber];
    }
}