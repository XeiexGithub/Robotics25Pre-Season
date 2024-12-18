package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmIO.ArmData;

public class Arm extends SubsystemBase {
    private ArmIO armIO = new ArmIO();
    private ArmData armData = new ArmData();
}

 @Override
  public void periodic() {
    armData.updateData(armIO);
    // simSystem.update(0.02);
    // position += (simSystem.getAngularVelocityRadPerSec() * 0.02);
    // SmartDashboard.putNumber("positionDeg", position * 180/Math.PI);
  }