package frc.robot.subsystems.turning;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
// 26.191, 137.594, 71.455, 186.943
public class TurningSpark implements TurningIO {
    private int absoluteCoderID;
    private int flywheekSparkMaxID;
    private int index;
    
    private final CANSparkMax flywheekSparkMax;
    private final CANcoder absoluteCoder;
    private final double encoderOffset;
    
    TurningSpark(int index) {
        this.index = index;
        flywheekSparkMaxID = Constants.TurningConstants.turningMotorIds[index];
        absoluteCoderID = Constants.TurningConstants.turningEncoderIds[index];
        encoderOffset = Constants.TurningConstants.turningEncoderOffsets[index] / 180 * Math.PI;
        flywheekSparkMax = new CANSparkMax(flywheekSparkMaxID, MotorType.kBrushless);
        absoluteCoder = new CANcoder(absoluteCoderID);
        
        flywheekSparkMax.setIdleMode(IdleMode.kBrake);
        // flywheekSparkMax.setInverted(true);
        System.out.println("Flywheel SparkMax instantiated");
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
        SmartDashboard.putNumber("applied volts: " + index, voltage);
    }

    @Override
    public void updateData(SwerveData data) {
        data.positionRad = getAbsoluteTurningPostionRad();
    }
}
