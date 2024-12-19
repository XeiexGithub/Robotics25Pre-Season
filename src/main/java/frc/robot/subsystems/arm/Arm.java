package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmIO.ArmData;
import frc.robot.subsystems.turning.TurningIO;
import frc.robot.subsystems.turning.TurningIO.SwerveData;

public class Arm extends SubsystemBase {
  private ArmIO armIO;
  private ArmData armData;

  PIDController pidController = new PIDController(ArmConstants.armControl.armkP, 0, ArmConstants.armControl.armkD);
  ArmFeedforward armFeedForward = new ArmFeedforward(ArmConstants.armControl.armkS, ArmConstants.armControl.armkG, ArmConstants.armControl.armkV, ArmConstants.armControl.armkA);
  
  public Arm() {
    if (Robot.isSimulation()) {
      armIO = new ArmSim();
      armData = new ArmData();
    }
  }
  
  public double getPosition(){
    return armData.positionRad; 
  }

  public void moveToSetpoint(double setpoint){
    armIO.setVoltage(pidController.calculate(getPosition(), setpoint));
  }

  public void stop() {
    armIO.setVoltage(0);
  }

  @Override
  public void periodic() {
    armIO.updateData(armData);
    if (armData.positionRad == 170 || armData.positionRad == -170 ){
      stop();
    }
  }
}