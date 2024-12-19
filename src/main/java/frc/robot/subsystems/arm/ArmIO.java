package frc.robot.subsystems.arm;

public interface ArmIO {
    public static class ArmData {
        double positionRad = 0;
        double positionReal = 0;
        double velocity = 0;
    }

    public default void updateData(ArmData data){
    }
    public default void setVoltage(double voltage){

    }
}
