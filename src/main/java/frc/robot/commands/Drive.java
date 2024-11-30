// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Wheel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** An example command that uses an example subsystem. */
public class Drive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private double drivesetpoint;
  private double kP;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Drive(double drivesetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivesetpoint = drivesetpoint;
    addRequirements(Robot.wheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Running");
    Robot.wheel.setMotorVoltage(0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopped");
    Robot.wheel.setMotorVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
