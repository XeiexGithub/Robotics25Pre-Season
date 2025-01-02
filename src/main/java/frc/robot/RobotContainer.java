// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drive;
// import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.MoveToSetpoint;
import frc.robot.commands.Turn;
// import frc.robot.commands.Turn;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final Wheel wheel = new Wheel();
  // private final Drive drivercmd = new Drive();

  private final Turn turn180 = new Turn(Math.PI);
  private final Turn turn90 = new Turn(Math.PI / 2);
  private final MoveToSetpoint arm45 = new MoveToSetpoint(Math.PI/4);
  private final MoveToSetpoint arm90 = new MoveToSetpoint(Math.PI/2);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController controller = new CommandXboxController(0);

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
    // controller.a().onTrue(turn180);
    // controller.b().onTrue(turn90);
    controller.a().onTrue(turn90);
    controller.b().onTrue(turn180);
    controller.x().onTrue(arm45);
    controller.y().onTrue(arm90);
    Robot.wheel.setDefaultCommand(new Drive(() -> controller.getLeftY()));
    // controller.x().onTrue(Commands.runOnce(() -> Robot.wheel.setMotorVoltage(12), Robot.wheel))
    // .onFalse(Commands.runOnce(() -> Robot.wheel.setMotorVoltage(0), Robot.wheel));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
