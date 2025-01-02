// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.driving;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivingConstants;
import frc.robot.Robot;
import frc.robot.subsystems.driving.DrivingIO.DriveData;
import frc.robot.subsystems.turning.TurningIO;
import frc.robot.subsystems.turning.TurningIO.SwerveData;

public class Driving extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  // private final CANSparkMax motor = new CANSparkMax(22, MotorType.kBrushless);

  private DrivingIO[] driveio = new DrivingIO[4];
  private DriveData[] data = new DriveData[4];

  public Driving() {
    if (Robot.isSimulation()){
      for (int i = 0; i < 4; i++){
        driveio[i] = new DrivingSim(i);
        data[i] = new DriveData();
      }
    } else {
      for (int i = 0; i < 4; i++){
        driveio[i] = new DrivingSparkMax(i);
      }
    }
  }

  public double getPosition(int index){
    return data[index].positionRad; 
  }

  // public double getVelocity(){
  //   return data.velocity;
  // }
  
  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void setMotorVoltage(int index, double setVelocity) {
    double speed = DrivingConstants.drivingkV * setVelocity;
    driveio[index].setVoltage(speed);
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
    for (int i = 0; i < 4; i++){
      driveio[i].updateData(data[i]);
      SmartDashboard.putNumber("drivePositionDeg: " + i, data[i].positionRad * 180/Math.PI);
      SmartDashboard.putNumber("drivePostionReal: " + i, data[i].positionReal);
    }
    // SmartDashboard.putNumber("driveSetVelocity", data.driveSetVelocity);
    // SmartDashboard.putNumber("voltsOut", voltOut);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
