package frc.robot.subsystems.turning;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import java.util.Random;

public class TurningSim implements TurningIO {
    private final FlywheelSim simSystem = new FlywheelSim(
        DCMotor.getNEO(1), 6, 0.04);

    private int index;
    
    Random rand = new Random();
    private double positionRad = rand.nextDouble(Math.PI * 2);;
    
    TurningSim(int index) {
        System.out.println("FlywheelSim instantiated");
        this.index = index;
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
        // voltage = MathUtil.applyDeadband(voltage, 0.05);
        simSystem.setInputVoltage(voltage);
        SmartDashboard.putNumber("applied volts: " + index, voltage);
    } 
}
