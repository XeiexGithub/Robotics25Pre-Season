package frc.robot.subsystems.driving;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;

public class DrivingSim implements DrivingIO {
    private final FlywheelSim driveSimSystem = new FlywheelSim(
        DCMotor.getNEO(1), 6, 0.04);
    
    private double positionRad = 0;
    
    DrivingSim(int index) {
        System.out.println("DriveSim instantiated");
    }
    
    @Override
    public void updateData(DriveData data) {
        driveSimSystem.update(OperatorConstants.loopPeriodSec);
        positionRad += (driveSimSystem.getAngularVelocityRadPerSec() * 0.02);
        data.positionRad = positionRad;
        data.positionReal = positionRad * Units.inchesToMeters(Constants.DrivingConstants.driveWheelRadius);
    }

    @Override
    public void setVoltage(double voltage){
        if (voltage < -12){
            voltage = -12;
        } else if (voltage > 12 ){
            voltage = 12;
        }
        driveSimSystem.setInputVoltage(voltage);
        SmartDashboard.putNumber("drive applied volts", voltage);
    } 
}
