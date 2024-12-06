// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.driving;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.driving.DrivingIO.DriveData;
import frc.robot.subsystems.turning.TurningIO;
import frc.robot.subsystems.turning.TurningIO.SwerveData;

public class Driving extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  // private final CANSparkMax motor = new CANSparkMax(22, MotorType.kBrushless);

  private DrivingIO driveio;
  private DriveData data = new DriveData();

  public Driving() {
    if (Robot.isSimulation()){
      driveio = new DrivingSim();
    } else {
      driveio = new DrivingSparkMax();
    }
  }

  public double getPosition(){
    return data.positionRad; 
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void setMotorVoltage(double speed) {
    driveio.setVoltage(speed);
  }

  public void stop() {
    
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    driveio.updateData(data);
    SmartDashboard.putNumber("drivePositionDeg", data.positionRad * 180/Math.PI);
    SmartDashboard.putNumber("drivePostionReal", data.positionReal);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
