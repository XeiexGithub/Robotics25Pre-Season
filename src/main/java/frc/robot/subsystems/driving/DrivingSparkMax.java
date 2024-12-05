package frc.robot.subsystems.driving;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
// 26.191, 137.594, 71.455, 186.943
public class DrivingSparkMax implements SwerveIO {
    private int absoluteCoderID;
    private int flywheekSparkMaxID;
    
    private final CANSparkMax drivingSparkMax;
    private final CANcoder drivingAbsoluteCoder;
    // private final double encoderOffset;
    
    DrivingSparkMax() {
        // flywheekSparkMaxID = Constants.TurningConstants.turningMotorIds[index];
        // absoluteCoderID = index + 11;
        // encoderOffset = Constants.TurningConstants.turningEncoderOffsets[index] / 180 * Math.PI;
        drivingSparkMax = new CANSparkMax(3, MotorType.kBrushless);
        drivingAbsoluteCoder = new CANcoder(4);
        System.out.println("Driving SparkMax instantiated");
        // FlywheekSparkMax.getEncoder().setPositionConversionFactor(1/Constants.TurningConstants.turningGearRatio * 2 * (Math.PI));
    }
    
    private double getAbsoluteTurningPostionRad() {
        double pos = Units.rotationsToRadians(absoluteCoder.getPosition().getValueAsDouble()) - encoderOffset; 
        while (pos < 0) {
            pos += Math.PI * 2;
        }
        while (pos > 2 * Math.PI){
            pos -= 2 * Math.PI;
        }
        return pos;
    }

    @Override
    public void setVoltage(double voltage){
        if (voltage < -12){
            voltage = -12;
        } else if (voltage > 12 ){
            voltage = 12;
        }
        flywheekSparkMax.setVoltage(voltage);
        SmartDashboard.putNumber("applied volts", voltage);
    }

    @Override
    public void updateData(SwerveData data) {
        data.positionRad = getAbsoluteTurningPostionRad();
    }
}
