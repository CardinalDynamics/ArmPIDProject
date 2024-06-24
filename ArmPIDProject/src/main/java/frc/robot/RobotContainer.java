// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Arm m_arm = new Arm();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_controller =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_arm.setDefaultCommand(
      Commands.run(() -> m_arm.setArmSpeed(0), m_arm)
    );
    m_controller.y().whileTrue(Commands.run(() -> m_arm.setArmSpeed(.35), m_arm));
    m_controller.x().whileTrue(Commands.run(() -> m_arm.setArmSpeed(-.25), m_arm));

  // Move the arm to 2 radians above horizontal when the 'A' button is pressed.
  m_controller.a().onTrue(
      Commands.runOnce(() -> {
            m_arm.setGoal(2);
            m_arm.enable();
          }, m_arm));

  // Move the arm to neutral position when the 'B' button is pressed.
  m_controller.b().onTrue(
      Commands.runOnce(() -> {
            m_arm.setGoal(74.5);
            m_arm.enable();
          },
          m_arm));

    // Disable the arm controller when Y is pressed.
    m_controller.y().onTrue(Commands.runOnce(m_arm::disable));
    
    m_controller.b().onTrue(new InstantCommand(() -> m_arm.setPID(), m_arm));

    m_controller.rightBumper().whileTrue(Commands.run(() -> m_arm.setGoal(m_arm.getGoal() + .01)));
    m_controller.leftBumper().whileTrue(Commands.run(() -> m_arm.setGoal(m_arm.getGoal() - .01)));
    m_controller.leftStick().onTrue(new InstantCommand(() -> m_arm.setGoal(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // }
}
