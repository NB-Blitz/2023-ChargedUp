package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SDSModuleConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double xyDeadband = 0.1;
    public static final double twistDeadband = 0.4;

    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final SDSModuleConstants chosenModule =
            SDSModuleConstants.SDSMK4(SDSModuleConstants.driveGearRatios.SDSMK4_L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(18.5);
        public static final double wheelBase = Units.inchesToMeters(19.5);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleStallLimit = 90;
        //public static final int angleFreeLimit = 50;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveStallLimit = 60;
        public static final int driveFreeLimit = 35;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.1;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.24;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.16;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Module Measurements **/
        public static final double wheelDiameter = 0.10033;
        public static final double driveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        public static final boolean driveInverted = true;
        public static final double steerReduction = (15.0 / 32.0) * (10.0 / 60.0);
        public static final boolean steerInverted = true;

        /** Meters per Second */
        //public static final double maxSpeed = 5880.0 / 60.0 * driveReduction * wheelDiameter * Math.PI;
        public static final double maxSpeed = 14.5 / 3.281;
        /** Radians per Second */
        //public static final double maxAngularVelocity = maxSpeed /
        //    Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
        public static final double maxAngularVelocity = Math.PI * 2.0;

        /* Neutral Modes */
        public static final IdleMode angleIdleMode = IdleMode.kBrake;
        public static final IdleMode driveIdleMode = IdleMode.kBrake;

        /* Module Specific Constants */
        private static double frontLeftOffset = 110.7;
        private static double frontRightOffset = 241.9;
        private static double backLeftOffset = 199.68;
        private static double backRightOffset = 25.5;

        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 16;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(frontLeftOffset);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 15;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(frontRightOffset);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 14;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(backLeftOffset);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(backRightOffset);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class Manipulator {
        public static final double SHOULDER_ANGLE_ERROR = 1;
        public static final double WRIST_ANGLE_ERROR = 1;
        public static final double TELESCOPE_ERROR = 5;

        /*************** Shoulder ***************/
        public static final int SHOULDER_MOTOR_ID = 9;

        public static final double SHOULDER_SPEED_FAST_MULT = 0.9;
        public static final double SHOULDER_SPEED_SLOW_MULT = 0.3;
        public static final double SHOULDER_ANGLE_SCALE = 1 / 2.017;
        public static final double MAX_SHOULDER_ANGLE = 110;


        /*************** Telescope ***************/
        public static final int TELESCOPE_MOTOR_ID = 10;

        public static final double TELESCOPE_SPEED_FAST_MULT = 0.8;
        public static final double TELESCOPE_SPEED_SLOW_MULT = 0.3;
        public static final double MAX_TELESCOPE_ENCODER_VALUE = 180.0;


        /*************** Wrist ***************/
        public static final int WRIST_MOTOR_ID = 11;
        public static final int WRIST_ENCODER_ID = 17;

        public static final double WRIST_SPEED_FAST_MULT = 1.0;
        public static final double WRIST_SPEED_SLOW_MULT = 0.25;
        public static final double WRIST_ENCODER_OFFSET = 157.0;


        /*************** Grip ***************/
        public static final int GRIP_MOTOR_ID = 12;
        
        public static final double GRIP_SPEED_MULTIPLIER = 1.0;


        /*************** Manipulator Positions ***************/
        public static final double SHOULDER_HOME = 0;
        public static final double TELESCOPE_HOME = 0;

        public static final double SHOULDER_FLOOR = 26.16;
        public static final double TELESCOPE_FLOOR = 65.69;

        public static final double SHOULDER_LVL2 = 89;
        public static final double TELESCOPE_LVL2 = 30;

        public static final double SHOULDER_LVL3 = 103.40;
        public static final double TELESCOPE_LVL3 = 180.0;

        public static final double SHOULDER_PLAYER = 89;
        public static final double TELESCOPE_PLAYER = 0;
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
