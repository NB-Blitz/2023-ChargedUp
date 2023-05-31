package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public final class SwerveConfigs {
    public CANCoderConfiguration swerveCanCoderConfig;

    public SwerveConfigs(){
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }

    public void configSparkMax(CANSparkMax sparkMax, String type) {
        if (type == "angle") {
            double positionConversionFactor = 360.0 / Constants.Swerve.angleGearRatio;
            sparkMax.getEncoder().setPositionConversionFactor(positionConversionFactor);
            sparkMax.getEncoder().setVelocityConversionFactor(positionConversionFactor / 60.0);

            sparkMax.getPIDController().setP(Constants.Swerve.angleKP);
            sparkMax.getPIDController().setI(Constants.Swerve.angleKI);
            sparkMax.getPIDController().setD(Constants.Swerve.angleKD);
            sparkMax.getPIDController().setFF(Constants.Swerve.angleKF);

            sparkMax.getPIDController().setFeedbackDevice(sparkMax.getEncoder());

            sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
            sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
            sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);

            sparkMax.enableVoltageCompensation(12.6);
            sparkMax.setSmartCurrentLimit(Constants.Swerve.angleStallLimit/*, Constants.Swerve.angleFreeLimit*/);
        } else if (type == "drive") {
            double positionConversionFactor = Math.PI * Constants.Swerve.wheelDiameter / Constants.Swerve.driveGearRatio;
            sparkMax.getEncoder().setPositionConversionFactor(positionConversionFactor);
            sparkMax.getEncoder().setVelocityConversionFactor(positionConversionFactor / 60.0);

            sparkMax.getPIDController().setP(Constants.Swerve.driveKP);
            sparkMax.getPIDController().setI(Constants.Swerve.driveKI);
            sparkMax.getPIDController().setD(Constants.Swerve.driveKD);
            sparkMax.getPIDController().setFF(Constants.Swerve.driveKF);

            sparkMax.getPIDController().setFeedbackDevice(sparkMax.getEncoder());

            sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
            sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
            sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);

            sparkMax.setSmartCurrentLimit(Constants.Swerve.driveStallLimit, Constants.Swerve.driveFreeLimit);

            sparkMax.setOpenLoopRampRate(Constants.Swerve.openLoopRamp);
            sparkMax.setClosedLoopRampRate(Constants.Swerve.closedLoopRamp);
        }
    }
}