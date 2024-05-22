package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmPIDConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    CANSparkMax m_leftArm;
    CANSparkMax m_rightArm;
    DutyCycleEncoder m_revEncoder;
    double setpoint;
    PIDController m_PIDController;
    ArmFeedforward m_Feedforward;
    

    // constructor
    public Arm() {
        m_leftArm = new CANSparkMax(kArmID, MotorType.kBrushless);
        m_rightArm = new CANSparkMax(kSecondArmID, MotorType.kBrushless);
        m_leftArm.setInverted(false);
        m_rightArm.setInverted(true);

        m_PIDController = new PIDController(kP, kI, kD);
        m_PIDController.setTolerance(kTolerance);

        m_Feedforward = new ArmFeedforward(kS, kG, kV);

        m_revEncoder = new DutyCycleEncoder(kEncoderPort);
        m_revEncoder.setConnectedFrequencyThreshold(500);
        m_revEncoder.reset();
        setpoint = 0;
    }

    public double getMeasurement() {
        return 75.0 - (m_revEncoder.get() * 360.0);
    }

    public boolean encoderConnected() {
        return m_revEncoder.isConnected();
    }

    public boolean atSetpoint() {
        return m_PIDController.atSetpoint();
    }

    public void setArmVoltage(double voltage) {
        m_rightArm.setVoltage(voltage);
        m_leftArm.setVoltage(voltage);
    }

    public void setArmSpeed(double speed) {
        m_leftArm.set(speed);
        m_leftArm.set(speed);
    }

    public void zeroEncoder() {
        m_revEncoder.reset();
    }

    public void setSetPoint(double setpoint) {
        m_PIDController.setSetpoint(setpoint);
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setPID(double p, double i, double d, double s, double g, double v) {
        m_PIDController.setP(p);
        m_PIDController.setI(i);
        m_PIDController.setD(d);
        m_Feedforward = new ArmFeedforward(s, g, v);
    }

    public void usePIDOutput(double newSetpoint) {
        setSetPoint(newSetpoint);
        setArmVoltage(m_PIDController.calculate(getMeasurement()) + m_Feedforward.calculate(setpoint * Math.PI / 180.0, 0));
    }
}
