package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmSim implements ArmIO {
    private final SingleJointedArmSim armSimSystem = new SingleJointedArmSim(DCMotor.getNEO(1), 6, 0.04);
}
