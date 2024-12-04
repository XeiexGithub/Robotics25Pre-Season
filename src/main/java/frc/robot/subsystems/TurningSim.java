package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;

public class TurningSim implements SwerveIO {
    private final FlywheelSim simSystem = new FlywheelSim(
        DCMotor.getNEO(1), 6, 0.04);

    private double positionRad = 0;
    
    TurningSim() {
        System.out.println("FlywheelSim instantiated");
    }
    
    @Override
    public void updateData(SwerveData data) {
        simSystem.update(OperatorConstants.loopPeriodSec);
        positionRad += (simSystem.getAngularVelocityRadPerSec() * 0.02);
        data.positionRad = positionRad;
    }

    @Override
    public void setVoltage(double voltage){
        if (voltage < -12){
            voltage = -12;
        } else if (voltage > 12 ){
            voltage = 12;
        }
        simSystem.setInputVoltage(voltage); // ????
        SmartDashboard.putNumber("applied volts", voltage);
    } 


   
}
