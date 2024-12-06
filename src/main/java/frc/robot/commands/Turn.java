// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.driving.Driving;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** An example command that uses an example subsystem. */
public class Turn extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private double setpoint;
  private double error;
  private PIDController pidController = new PIDController(Constants.TurningConstants.turningkP, 0,
      Constants.TurningConstants.turningkD);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Turn(double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.setpoint = setpoint;
    pidController.enableContinuousInput(0, 2 * Math.PI);
    addRequirements(Robot.flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Have a Command to set the position of the flywheel! Use PID!
    SmartDashboard.putNumber("kP", Constants.TurningConstants.turningkP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (int i = 0; i < 4; i++) {
      double proportionOutput = pidController.calculate(Robot.flywheel.getPosition(i), setpoint);
      Robot.flywheel.setMotorVoltage(proportionOutput, i);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    for (int i = 0; i < 4; i++) {
      Robot.flywheel.setMotorVoltage(0, i);
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
