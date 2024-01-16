package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;

public class GripSubsystem extends SubsystemBase {
    private final WPI_TalonSRX gripMotor;
    private final TalonSRXSimCollection gripMotorSim;

    private double gripSpeed;

    public GripSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Manipulator");

        this.gripMotor = new WPI_TalonSRX(Manipulator.GRIP_MOTOR_ID);
        this.gripMotorSim = this.gripMotor.getSimCollection();

        tab.addDouble("Grip Pos", () -> getGripPos());
    }

    public void move(double gripSpeed) {
        this.gripSpeed = gripSpeed;
    }

    @Override
    public void periodic() {
        if (gripSpeed > 0 /*&& getGripPos() < MAX_GRIP_ENCODER_VALUE*/) {
            gripMotor.set(TalonSRXControlMode.PercentOutput, gripSpeed * Manipulator.GRIP_SPEED_MULTIPLIER);
        } else if (gripSpeed < 0 /*&& getGripPos() > MIN_GRIP_ENCODER_VALUE*/) {
            gripMotor.set(TalonSRXControlMode.PercentOutput, gripSpeed * Manipulator.GRIP_SPEED_MULTIPLIER);
        } else {
            gripMotor.set(TalonSRXControlMode.PercentOutput, 0);
        }
    }

    private double getGripPos() {
        return this.gripMotor.getSelectedSensorPosition();
    }
}
