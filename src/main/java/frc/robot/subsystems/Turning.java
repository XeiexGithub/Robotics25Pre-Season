// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveIO.SwerveData;

public class Turning extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    private SwerveIO[] swerveio;
    private SwerveData[] data;
    public Turning() {
      if (Robot.isSimulation()) {
        swerveio[0] = new TurningSim();
      } else {
        for (int i = 0; i < 4; i++){
          swerveio[i] = new TurningSpark(i);
          data[i] = new SwerveData();
        }
      }
    }

    public double getPosition(int index){
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
    for (int i = 0; i < 4; i++){
      swerveio[i].updateData(data[i]);
      SmartDashboard.putNumber("positionDeg: " + i, data[i].positionRad * 180/Math.PI);
    }
    // simSystem.update(0.02);
    // position += (simSystem.getAngularVelocityRadPerSec() * 0.02);
    // SmartDashboard.putNumber("positionDeg", position * 180/Math.PI);
  }

  @Override
  public void simulationPeriodic() {
  }
}