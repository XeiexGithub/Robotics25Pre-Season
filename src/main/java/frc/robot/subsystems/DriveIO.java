package frc.robot.subsystems;

public interface DriveIO {
    public static class DriveData {
        double positionRad = 0;
    }

    public default void updateData(DriveData data){
    };

    public default void setVoltage(double volts){
    };
}
