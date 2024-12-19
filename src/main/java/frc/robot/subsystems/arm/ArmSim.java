package frc.robot.subsystems.arm;

import java.util.Random;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;

public class ArmSim implements ArmIO {
    // Random rand = new Random();
    // private double positionRad = MathUtil.clamp(rand.nextDouble(Math.PI * 2), -170, 170);

    Mechanism2d mech = new Mechanism2d(3, 3);
    MechanismRoot2d root = mech.getRoot("arm", 2, 0);

    private SingleJointedArmSim armSimSystem = new SingleJointedArmSim(
        DCMotor.getNEO(2),
        333.333,
        0.755, //2578.65
        Units.inchesToMeters(21.1), //  21.1 in
        Units.degreesToRadians(-170),
        Units.degreesToRadians(170),
        true,
        Units.degreesToRadians(0));

    @Override
    public void updateData(ArmData armData) {
        armSimSystem.update(OperatorConstants.loopPeriodSec);
        armData.positionRad = armSimSystem.getAngleRads();
    }

    @Override
    public void setVoltage(double voltage){
        if (voltage < -12){
            voltage = -12;
        } else if (voltage > 12 ){
            voltage = 12;
        }
        armSimSystem.setInputVoltage(voltage);
        SmartDashboard.putNumber("applied volts", voltage);
    }
}
