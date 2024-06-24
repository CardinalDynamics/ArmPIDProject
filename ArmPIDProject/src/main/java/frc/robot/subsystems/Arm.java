package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmPIDConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends ProfiledPIDSubsystem {
  
    DutyCycleEncoder m_revEncoder;
    ArmFeedforward m_feedforward;
    CANSparkMax m_leftArm;
    CANSparkMax m_rightArm;

    ShuffleboardTab tab = Shuffleboard.getTab("arm");
    GenericEntry p = tab.add("kP", 0).getEntry();
    GenericEntry i = tab.add("kI", 0).getEntry();
    GenericEntry d = tab.add("kD", 0).getEntry();

    /** Create a new ArmSubsystem. */
    public Arm() {
        super(
            new ProfiledPIDController(
                kP,
                kI,
                kD,
                new TrapezoidProfile.Constraints(
                    3, 10)),
                    0
            );
        // Start arm at rest in neutral position
        setGoal(74.5);
        m_feedforward = new ArmFeedforward(kS, .2, kV);
        m_revEncoder = new DutyCycleEncoder(0);
        m_leftArm = new CANSparkMax(kArmID, MotorType.kBrushless);
        m_rightArm = new CANSparkMax(kSecondArmID, MotorType.kBrushless);
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output
        m_leftArm.setVoltage(output + feedforward);
    }

    @Override
    public double getMeasurement() {
        return 74.5 - m_revEncoder.get();
    }

    public void setArmVoltage(double voltage) {
        m_rightArm.setVoltage(voltage);
        m_leftArm.setVoltage(voltage);
        SmartDashboard.putNumber("output", voltage);
    }

    public void setArmSpeed(double speed) {
        m_leftArm.set(speed);
        m_leftArm.set(speed);
    }

    public void zeroEncoder() {
        m_revEncoder.reset();
    }

    public void setPID() {
        super.m_controller.setP(p.getDouble(0));
        super.m_controller.setI(i.getDouble(0));
        super.m_controller.setD(d.getDouble(0));
        
    }

    public void resetEncoder() {
        m_revEncoder.reset();
    }

    public double getVeloSetpoint() {
        return super.m_controller.getSetpoint().velocity;
    }

    public double getGoal() {
        return m_controller.getGoal().position;
    }
}