package frc.robot.subsystems.arm;

import java.util.Random;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;

public class ArmSim implements ArmIO {
    // Random rand = new Random();
    // private double positionRad = -170 * Math.PI / 180 + rand.nextDouble() * (2 * 170 * Math.PI / 180);
    private double positionRad = 0;

    public Mechanism2d mech = new Mechanism2d(3, 3);
    MechanismRoot2d root = mech.getRoot("arm", 2, 0);
    MechanismLigament2d armMech = root.append(new MechanismLigament2d("arm", Units.inchesToMeters(21.1), positionRad * 180/Math.PI));

    private SingleJointedArmSim armSimSystem = new SingleJointedArmSim(
        DCMotor.getNEO(2),
        333.333,
        0.755,
        Units.inchesToMeters(21.1),
        Units.degreesToRadians(-170),
        Units.degreesToRadians(170),
        false,
        positionRad);

    @Override
    public void updateData(ArmData armData) {
        armSimSystem.update(OperatorConstants.loopPeriodSec);
        armData.positionRad = armSimSystem.getAngleRads();
        armData.positionDeg = armData.positionRad * 180/Math.PI;
        armMech.setAngle(armData.positionDeg);
    }

    @Override
    public void setVoltage(double voltage){
        if (voltage < -12){
            voltage = -12;
        } else if (voltage > 12 ){
            voltage = 12;
        }
        armSimSystem.setInputVoltage(voltage);
        SmartDashboard.putNumber("arm applied volts", voltage);
    }
}
