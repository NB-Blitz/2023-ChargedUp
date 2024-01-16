package frc.robot;

//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.util.REVUtils;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.jni.RevJNIWrapper;

import com.revrobotics.REVPhysicsSim;
import com.ctre.phoenix.sensors.CANCoderSimCollection;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;
    private WPI_CANCoder angleEncoder;
    private CANCoderSimCollection simCoder;

    //SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new WPI_CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        configDriveMotor();

        lastAngle = getState().angle;

        if (RobotBase.isSimulation()) {
          REVPhysicsSim.getInstance().addSparkMax(mAngleMotor, DCMotor.getNEO(1));
          REVPhysicsSim.getInstance().addSparkMax(mDriveMotor, DCMotor.getNEO(1));
          simCoder = angleEncoder.getSimCollection();
        }
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = REVUtils.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);

        if (RobotBase.isSimulation()){
            //simUpdateDrivePosition(desiredState);
            //simTurnPosition(desiredState.angle);
        }
    }

    private void simUpdateDrivePosition(SwerveModuleState state) {
        int simVelocity =(int)(state.speedMetersPerSecond/10);
        simCoder.setVelocity(simVelocity);
        int distancePer20Ms = simVelocity / 5;
        simCoder.addPosition(distancePer20Ms);
    }

    /*private void simTurnPosition(double angle) {
        if (angle != m_currentAngle && m_simTurnAngleIncrement == 0) {
            m_simAngleDifference = angle - m_currentAngle;
            m_simTurnAngleIncrement = m_simAngleDifference / 20.0;// 10*20ms = .2 sec move time
        }
      
        if (m_simTurnAngleIncrement != 0) {
            m_currentAngle += m_simTurnAngleIncrement;
      
            if ((Math.abs(angle - m_currentAngle)) < .1) {
                m_currentAngle = angle;
                m_simTurnAngleIncrement = 0;
            }
        }
    }*/

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput);
        }
        else {
            double velocity = desiredState.speedMetersPerSecond;
            mDriveMotor.getPIDController().setReference(velocity, ControlType.kVelocity);
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mAngleMotor.getPIDController().setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(mAngleMotor.getEncoder().getPosition());
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute() {
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        mAngleMotor.getEncoder().setPosition(absolutePosition);
    }

    private void configAngleEncoder() {        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.swerveConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        mAngleMotor.restoreFactoryDefaults();
        Robot.swerveConfigs.configSparkMax(mAngleMotor, "angle");
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setIdleMode(Constants.Swerve.angleIdleMode);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        mDriveMotor.restoreFactoryDefaults();
        Robot.swerveConfigs.configSparkMax(mDriveMotor, "drive");
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setIdleMode(Constants.Swerve.driveIdleMode);
        mDriveMotor.getEncoder().setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            mDriveMotor.getEncoder().getVelocity(),
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            mDriveMotor.getEncoder().getPosition(),
            getAngle()
        );
    }
}