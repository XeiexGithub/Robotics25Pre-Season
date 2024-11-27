// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Wheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** An example command that uses an example subsystem. */
public class Turn extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private double setpoint;
  private double kP = 0.4;

  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Turn(double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.setpoint = setpoint;
    addRequirements(Robot.simSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Have a Command to set the position of the flywheel! Use PID!
    SmartDashboard.putNumber("kP", kP);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = setpoint - Robot.flywheel.getPositionRad();
    double proportionOutput = error * kP;
    Robot.flywheel.setVoltage(proportionOutput);
    SmartDashboard.putNumber("error", error / (2 * Math.PI * 360));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.flywheel.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
