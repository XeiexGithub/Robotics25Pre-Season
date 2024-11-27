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

public class Turning extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    private final FlywheelSim simSystem = new FlywheelSim(
        DCMotor.getNEO(1), 6, 0.04);
    private double position = 0;

    public double getPosition(){
        return position;
    }

    public void setVoltage(double voltage){
        if (voltage < -12){
            voltage = -12;
        } else if (voltage > 12 ){
            voltage = 12;
        }
        simSystem.setInputVoltage(voltage);
        SmartDashboard.putNumber("Applied Volts", voltage);
    } 

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void setMotorVoltage(double speed) {
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
  }

  @Override
  public void simulationPeriodic() {
    simSystem.update(0.02);
    position += simSystem.getAngularVelocityRadPerSec();
  }
}
