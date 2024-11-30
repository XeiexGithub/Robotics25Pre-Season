package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurningSpark implements SwerveIO {
    private final CANSparkMax FlywheekSparkMax = new CANSparkMax(3, MotorType.kBrushless);

    private double positionRad = 0;
    
    public void SubsystemSim() {
        System.out.println("FlywheelSim instantiated");
    }
    
    @Override
    public void setVoltage(double voltage){
        if (voltage < -12){
            voltage = -12;
        } else if (voltage > 12 ){
            voltage = 12;
        }
        FlywheekSparkMax.setVoltage(voltage);
        SmartDashboard.putNumber("applied volts", voltage);
    } 
}
