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
public class DrivingSparkMax implements DrivingIO {
    // private int absoluteCoderID;
    // private int flywheekSparkMaxID;
    
    private final CANSparkMax drivingSparkMax;
    // private final double encoderOffset;
    
    DrivingSparkMax() {
        // flywheekSparkMaxID = Constants.TurningConstants.turningMotorIds[index];
        // absoluteCoderID = index + 11;
        // encoderOffset = Constants.TurningConstants.turningEncoderOffsets[index] / 180 * Math.PI;
        drivingSparkMax = new CANSparkMax(3, MotorType.kBrushless);
        System.out.println("Driving SparkMax instantiated");
        drivingSparkMax.getEncoder().setPositionConversionFactor(1/Constants.DrivingConstants.driveGearRatio * 2 * (Math.PI));
    }
    
    private double getAbsoluteTurningPostionRad() {
        double pos = Units.rotationsToRadians(drivingSparkMax.getEncoder().getPosition()); 
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
        drivingSparkMax.setVoltage(voltage);
        SmartDashboard.putNumber("drive applied volts", voltage);
    }

    @Override
    public void updateData(DriveData data) {
        double positionRad = getAbsoluteTurningPostionRad();
        data.positionRad = getAbsoluteTurningPostionRad();
        data.positionReal = positionRad * Units.inchesToMeters(Constants.DrivingConstants.driveWheelRadius);
    }
}
