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
    private int driveSparkMaxID;

    private final CANSparkMax drivingSparkMax;
    // private final double encoderOffset;

    DrivingSparkMax(int index) {
        driveSparkMaxID = Constants.DrivingConstants.driveMotorIds[index];;
        drivingSparkMax = new CANSparkMax(driveSparkMaxID, MotorType.kBrushless);
        System.out.println("Driving SparkMax instantiated");
        drivingSparkMax.getEncoder()
                .setPositionConversionFactor(1 / Constants.DrivingConstants.driveGearRatio * 2 * (Math.PI));
        drivingSparkMax.getEncoder()
                .setVelocityConversionFactor((1 / Constants.DrivingConstants.driveGearRatio * 2 * (Math.PI)) / 60.0);
    }

    @Override
    public void setVoltage(double voltage) {
        if (voltage < -12) {
            voltage = -12;
        } else if (voltage > 12) {
            voltage = 12;
        }
        drivingSparkMax.setVoltage(voltage);
        SmartDashboard.putNumber("drive applied volts", voltage);
    }

    @Override
    public void updateData(DriveData data) {
        data.positionRad = drivingSparkMax.getEncoder().getPosition();
        data.velocity = drivingSparkMax.getEncoder().getVelocity();
        data.positionReal = data.positionRad * Units.inchesToMeters(Constants.DrivingConstants.driveWheelRadius);
    }
}
