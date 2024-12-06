// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.Constants.DrivingConstants;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** An example command that uses an example subsystem. */
public class Drive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  double driveSetVelocity;
  private Supplier<Double> leftY;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Drive(Supplier<Double> leftY) {
    this.leftY = leftY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.wheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("kV", DrivingConstants.drivingkV);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSetVelocity = -leftY.get() * DrivingConstants.maxV;
    double voltOut = DrivingConstants.drivingkV * driveSetVelocity;
    Robot.wheel.setMotorVoltage(voltOut);
    SmartDashboard.putNumber("driveSetVelocity", driveSetVelocity);
    SmartDashboard.putNumber("voltsOut", voltOut);
    // SmartDashboard.putNumber("velocity", velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.wheel.setMotorVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
