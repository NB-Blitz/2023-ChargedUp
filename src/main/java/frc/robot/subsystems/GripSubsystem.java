package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class GripSubsystem extends SubsystemBase {
    private final TalonSRX gripMotor;

    private double gripSpeed;

    public GripSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Manipulator");

        this.gripMotor = new TalonSRX(Manipulator.GRIP_MOTOR_ID);

        tab.addDouble("Grip Pos", () -> getGripPos());
    }

    public void move(double gripSpeed) {
        this.gripSpeed = gripSpeed;
    }


    public void runGrip() {
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
