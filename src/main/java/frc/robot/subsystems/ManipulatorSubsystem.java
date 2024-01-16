package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static com.revrobotics.SparkMaxLimitSwitch.Type.*;
import static com.revrobotics.SparkMaxRelativeEncoder.Type.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import com.revrobotics.REVPhysicsSim;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.sensors.CANCoderSimCollection;


public class ManipulatorSubsystem extends SubsystemBase {
    private final CANSparkMax shoulderMotor;
    private final CANSparkMax telescopeMotor;
    private final WPI_TalonSRX wristMotor;

    private final RelativeEncoder shoulderEncoder;
    private final RelativeEncoder telescopeEncoder;
    private final WPI_CANCoder wristEncoder;

    private final ColorSensorV3 shoulderSensor;
    private final SparkMaxLimitSwitch telescopeSwitch;

    private boolean presetMode = false;
    private boolean killWrist = false;

    private double shoulderSpeed;
    private double telescopeSpeed;
    //private double wristSpeed;

    private double targetShoulderAngle;
    private double targetTelescopeLength;
    private double targetWristAngle;

    public ManipulatorSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Manipulator");

        shoulderMotor = new CANSparkMax(Manipulator.SHOULDER_MOTOR_ID, MotorType.kBrushless);
        telescopeMotor = new CANSparkMax(Manipulator.TELESCOPE_MOTOR_ID, MotorType.kBrushless);
        wristMotor = new WPI_TalonSRX(Manipulator.WRIST_MOTOR_ID);
        shoulderEncoder = shoulderMotor.getEncoder(kHallSensor, 42);
        telescopeEncoder = telescopeMotor.getEncoder(kHallSensor, 42);
        wristEncoder = new WPI_CANCoder(Manipulator.WRIST_ENCODER_ID);
        shoulderSensor = new ColorSensorV3(I2C.Port.kOnboard);
        telescopeSwitch = telescopeMotor.getReverseLimitSwitch(kNormallyClosed);

        REVPhysicsSim.getInstance().addSparkMax(shoulderMotor, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(telescopeMotor, DCMotor.getNEO(1));
        TalonSRXSimCollection wristMotorSim = wristMotor.getSimCollection();
        CANCoderSimCollection wristEncoderSim = wristEncoder.getSimCollection();

        shoulderEncoder.setPositionConversionFactor(Manipulator.SHOULDER_ANGLE_SCALE);

        wristMotor.setInverted(true);
        wristEncoder.configMagnetOffset(Manipulator.WRIST_ENCODER_OFFSET);

        tab.addDouble("Shoulder Angle", () -> getShoulderAngle());
        tab.addDouble("Telescope Pos", () -> getTelescopePos());
        tab.addDouble("Wrist Angle", () -> getWristAngle());

        tab.addDouble("Shoulder Red", () -> (double)shoulderSensor.getRawColor().red);
        tab.addBoolean("Telescope Switch", () -> telescopeSwitch.isPressed());
    }

    public void move(double shoulderSpeed, double telescopeSpeed, double wristSpeed) {
        if (shoulderSpeed != 0 || telescopeSpeed != 0 /*|| wristSpeed != 0*/) {
            presetMode = false;
        }
        this.shoulderSpeed = shoulderSpeed;
        this.telescopeSpeed = telescopeSpeed;
        //this.wristSpeed = wristSpeed;
    }

    public void setHome() { // Starting position
        presetMode = true;
        targetShoulderAngle = Manipulator.SHOULDER_HOME;
        targetTelescopeLength = Manipulator.TELESCOPE_HOME;
    }

    public void setFloor() { // Floor level
        presetMode = true;
        targetShoulderAngle = Manipulator.SHOULDER_FLOOR;
        targetTelescopeLength = Manipulator.TELESCOPE_FLOOR;
    }

    public void setTwo() { // 2nd level
        presetMode = true;
        targetShoulderAngle = Manipulator.SHOULDER_LVL2;
        targetTelescopeLength = Manipulator.TELESCOPE_LVL2;
    }

    public void setThree() { // 3rd level
        presetMode = true;
        targetShoulderAngle = Manipulator.SHOULDER_LVL3;
        targetTelescopeLength = Manipulator.TELESCOPE_LVL3;
    }

    public void setPlayerShelf() { // Human player shelf
        presetMode = true;
        targetShoulderAngle = Manipulator.SHOULDER_PLAYER;
        targetTelescopeLength = Manipulator.TELESCOPE_PLAYER;
    }

    @Override
    public void periodic() {
        if (isShoulderHome()) {
            shoulderEncoder.setPosition(0);
        }
        if (telescopeSwitch.isPressed()) {
            telescopeEncoder.setPosition(0);
        }

        if (!presetMode) {
            // Shoulder Manual Control
            if ((isShoulderHome() && shoulderSpeed <= 0) ||
                (getShoulderAngle() >= Manipulator.MAX_SHOULDER_ANGLE && shoulderSpeed >= 0) ||
                (getShoulderAngle() <= Manipulator.SHOULDER_FLOOR + Manipulator.SHOULDER_ANGLE_ERROR &&
                    shoulderSpeed < 0 && !telescopeSwitch.isPressed())) {
                shoulderMotor.set(0);
            } else if (shoulderSpeed < 0 && getShoulderAngle() < 30) {
                shoulderMotor.set(shoulderSpeed * Manipulator.SHOULDER_SPEED_SLOW_MULT);
            } else {
                shoulderMotor.set(shoulderSpeed * Manipulator.SHOULDER_SPEED_FAST_MULT);
            }

            // Telescope Manual Control
            if ((getTelescopePos() >= Manipulator.MAX_TELESCOPE_ENCODER_VALUE && telescopeSpeed > 0) ||
                (getShoulderAngle() < Manipulator.SHOULDER_FLOOR - Manipulator.SHOULDER_ANGLE_ERROR && telescopeSpeed > 0)) {
                telescopeMotor.set(0);
            } else if ((getTelescopePos() < 20 && telescopeSpeed < 0) ||
                       (getTelescopePos() > Manipulator.MAX_TELESCOPE_ENCODER_VALUE - 20 && telescopeSpeed > 0)) {
                telescopeMotor.set(telescopeSpeed * Manipulator.TELESCOPE_SPEED_SLOW_MULT);
            } else {
                telescopeMotor.set(telescopeSpeed * Manipulator.TELESCOPE_SPEED_FAST_MULT);
            }
        } else {
            // Use limit switches to go to home position instead of encoder values
            if (targetShoulderAngle == 0 && targetTelescopeLength == 0) {
                if (isShoulderHome() || !telescopeSwitch.isPressed()) {
                    shoulderMotor.set(0);
                } else if (getShoulderAngle() < 30) {
                    shoulderMotor.set(-Manipulator.SHOULDER_SPEED_SLOW_MULT);
                } else {
                    shoulderMotor.set(-Manipulator.SHOULDER_SPEED_FAST_MULT);
                }
    
                if (telescopeSwitch.isPressed()) {
                    telescopeMotor.set(0);
                } else if (getTelescopePos() < 20) {
                    telescopeMotor.set(-Manipulator.TELESCOPE_SPEED_SLOW_MULT);
                } else {
                    telescopeMotor.set(-Manipulator.TELESCOPE_SPEED_FAST_MULT);
                }
            } else {
                // Shoulder Preset Control
                if (getShoulderAngle() > targetShoulderAngle + Manipulator.SHOULDER_ANGLE_ERROR) {
                    if (getShoulderAngle() - targetShoulderAngle < 10) {
                        shoulderMotor.set(-Manipulator.SHOULDER_SPEED_SLOW_MULT);
                    } else {
                        shoulderMotor.set(-Manipulator.SHOULDER_SPEED_FAST_MULT);
                    }
                } else if (getShoulderAngle() < targetShoulderAngle - Manipulator.SHOULDER_ANGLE_ERROR) {
                    if (targetShoulderAngle - getShoulderAngle() < 10) {
                        shoulderMotor.set(Manipulator.SHOULDER_SPEED_SLOW_MULT);
                    } else {
                        shoulderMotor.set(Manipulator.SHOULDER_SPEED_FAST_MULT);
                    }
                } else {
                    shoulderMotor.set(0);
                }

                // Telescope Preset Control
                if (getShoulderAngle() < Manipulator.SHOULDER_FLOOR - Manipulator.SHOULDER_ANGLE_ERROR) {
                    telescopeMotor.set(0);
                } else if (getTelescopePos() > targetTelescopeLength + Manipulator.TELESCOPE_ERROR) {
                    if (getTelescopePos() - targetTelescopeLength < 20) {
                        telescopeMotor.set(-Manipulator.TELESCOPE_SPEED_SLOW_MULT);
                    } else {
                        telescopeMotor.set(-Manipulator.TELESCOPE_SPEED_FAST_MULT);
                    }
                } else if (getTelescopePos() < targetTelescopeLength - Manipulator.TELESCOPE_ERROR) {
                    if (targetTelescopeLength - getTelescopePos() < 20) {
                        telescopeMotor.set(Manipulator.TELESCOPE_SPEED_SLOW_MULT);
                    } else {
                        telescopeMotor.set(Manipulator.TELESCOPE_SPEED_FAST_MULT);
                    }
                } else {
                    telescopeMotor.set(0);
                }
            }
        }

        // Wrist Control
        if (presetMode && targetShoulderAngle == 0) {
            targetWristAngle = 0;
        } else if (getShoulderAngle() >= Manipulator.SHOULDER_FLOOR - Manipulator.SHOULDER_ANGLE_ERROR) {
            targetWristAngle = 90 + getShoulderAngle();
        } else {
            targetWristAngle = 0;
        }

        if (!killWrist) {
            if (targetWristAngle == 0) {
                if (getWristAngle() > targetWristAngle + Manipulator.WRIST_ANGLE_ERROR && getWristAngle() < 360 - Manipulator.WRIST_ANGLE_ERROR) {
                    if (getWristAngle() > 270) {
                        wristMotor.set(TalonSRXControlMode.PercentOutput, -Manipulator.WRIST_SPEED_SLOW_MULT);
                    } else if (getWristAngle() > 10) {
                        wristMotor.set(TalonSRXControlMode.PercentOutput, Manipulator.WRIST_SPEED_FAST_MULT);
                    } else {
                        wristMotor.set(TalonSRXControlMode.PercentOutput, Manipulator.WRIST_SPEED_SLOW_MULT);
                    }
                } else {
                    wristMotor.set(TalonSRXControlMode.PercentOutput, 0);
                }
            } else {
                if (getWristAngle() > targetWristAngle + Manipulator.WRIST_ANGLE_ERROR) {
                    if (getWristAngle() > 270) {
                        wristMotor.set(TalonSRXControlMode.PercentOutput, -Manipulator.WRIST_SPEED_SLOW_MULT);
                    } else if (getWristAngle() - targetWristAngle > 10) {
                        wristMotor.set(TalonSRXControlMode.PercentOutput, Manipulator.WRIST_SPEED_FAST_MULT);
                    } else {
                        wristMotor.set(TalonSRXControlMode.PercentOutput, Manipulator.WRIST_SPEED_SLOW_MULT);
                    }
                } else if (getWristAngle() < targetWristAngle - Manipulator.WRIST_ANGLE_ERROR) {
                    if (targetWristAngle - getWristAngle() > 10) {
                        wristMotor.set(TalonSRXControlMode.PercentOutput, -Manipulator.WRIST_SPEED_FAST_MULT);
                    } else { 
                        wristMotor.set(TalonSRXControlMode.PercentOutput, -Manipulator.WRIST_SPEED_SLOW_MULT);
                    }
                } else {
                    wristMotor.set(TalonSRXControlMode.PercentOutput, 0);
                }
            }
        } else {
            wristMotor.set(TalonSRXControlMode.PercentOutput, 0);
        }
        //wristMotor.set(TalonSRXControlMode.PercentOutput, wristSpeed * WRIST_SPEED_FAST_MULT);
    }

    private double getShoulderAngle() {
        return shoulderEncoder.getPosition() /** SHOULDER_ANGLE_SCALE*/;
    }

    private boolean isShoulderHome() {
        return shoulderSensor.getRawColor().red > 700;
    }

    private double getTelescopePos() {
        return telescopeEncoder.getPosition();
    }

    private double getWristAngle() {
        return wristEncoder.getAbsolutePosition();
    }

    public void killWrist(boolean kill) {
        killWrist = kill;
    }

    public boolean atTargetPreset() {
        return getShoulderAngle() >= targetShoulderAngle - Manipulator.SHOULDER_ANGLE_ERROR &&
               getShoulderAngle() <= targetShoulderAngle + Manipulator.SHOULDER_ANGLE_ERROR &&
               getTelescopePos() >= targetTelescopeLength - Manipulator.TELESCOPE_ERROR &&
               getTelescopePos() <= targetTelescopeLength + Manipulator.TELESCOPE_ERROR;
    }
}
