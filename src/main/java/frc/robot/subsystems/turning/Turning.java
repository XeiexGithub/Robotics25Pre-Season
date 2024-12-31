// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turning;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.driving.DrivingIO.DriveData;
import frc.robot.subsystems.turning.TurningIO.SwerveData;

public class Turning extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private TurningIO[] swerveio = new TurningIO[4];
  private SwerveData[] data = new SwerveData[4];

  public Turning() {
    if (Robot.isSimulation()) {
      for (int i = 0; i < 4; i++) {
        swerveio[i] = new TurningSim(i);
        data[i] = new SwerveData();
      }
    } else {
      for (int i = 0; i < 4; i++) {
        swerveio[i] = new TurningSpark(i);
        data[i] = new SwerveData();
      }
    }
  }

  public double getPosition(int index) {
    return data[index].positionRad;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void setMotorVoltage(double speed, int index) {
    swerveio[index].setVoltage(speed);
  }

  public void stop(int index) {
    swerveio[index].setVoltage(0);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    double[] positions = new double[8];
    for (int i = 0; i < 4; i++) {
      swerveio[i].updateData(data[i]);
      positions[i * 2] = data[i].positionRad * 180 / Math.PI;
    }
    SmartDashboard.putNumberArray("Swerve States", positions);

    // simSystem.update(0.02);
    // position += (simSystem.getAngularVelocityRadPerSec() * 0.02);
    // SmartDashboard.putNumber("positionDeg", position * 180/Math.PI);
  }

  @Override
  public void simulationPeriodic() {
  }
}