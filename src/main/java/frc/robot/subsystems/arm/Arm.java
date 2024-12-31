package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmIO.ArmData;
import frc.robot.utils.ShuffleData;

public class Arm extends SubsystemBase {
  private ArmSim armIO;
  private ArmData armData;

  private ShuffleData<Double> kPData = new ShuffleData<Double>(this.getName(), "kPData", ArmConstants.armControl.armkP);
  private ShuffleData<Double> kDData = new ShuffleData<Double>(this.getName(), "kDData", ArmConstants.armControl.armkD);

  // PIDController pidController = new PIDController(ArmConstants.armControl.armkP, 0, ArmConstants.armControl.armkD);
  PIDController pidController = new PIDController(ArmConstants.armControl.armkP, 0, ArmConstants.armControl.armkD);

  public Arm() {
    if (Robot.isSimulation()) {
      armIO = new ArmSim();
      armData = new ArmData(); 
    }
    else {
      // change later
      armIO = new ArmSim();
      armData = new ArmData(); 
    }
  }
  
  public double getPosition(){
    return armData.positionRad; 
  }

  public void moveToSetpoint(double setpoint){
    armIO.setVoltage(pidController.calculate(getPosition(), setpoint) + ArmConstants.armControl.armkG);
  }

  public void stop() {
    armIO.setVoltage(0);
  }

  @Override
  public void periodic() {
    armIO.updateData(armData);
    SmartDashboard.putNumber("arm positionDeg", armData.positionDeg);
    SmartDashboard.putData("arm mechanism", armIO.mech);
    pidController.setP(kPData.get());
    pidController.setD(kDData.get());
  }
}