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
import frc.robot.Robot;
import frc.robot.subsystems.SwerveIO.SwerveData;

public class Turning extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    private SwerveIO swerveio;
    private final FlywheelSim simSystem = new FlywheelSim(
        DCMotor.getNEO(1), 6, 0.04);

    private SwerveData data = new SwerveData();
    public double positionDeg;
    
    public Turning() {
      if (Robot.isSimulation()) {
        swerveio = new TurningSim();
      } else {
        swerveio = new TurningSpark();
      }
    }

    public double getPosition(){
        return data.positionRad; // how do i get this 
    }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void setMotorVoltage(double speed) {
    swerveio.setVoltage(speed);
  }

  public void stop() {
    swerveio.setVoltage(0);
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
    // simSystem.update(0.02);
    // position += (simSystem.getAngularVelocityRadPerSec() * 0.02);
    // SmartDashboard.putNumber("positionDeg", position * 180/Math.PI);
  }

  @Override
  public void simulationPeriodic() {
    swerveio.updateData(data);
    data.positionRad += (simSystem.getAngularVelocityRadPerSec() * 0.02);
    positionDeg = data.positionRad * 180/Math.PI;
    SmartDashboard.putNumber("positionDeg", positionDeg);
  }
}