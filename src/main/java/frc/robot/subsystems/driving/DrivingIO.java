package frc.robot.subsystems.driving;

public interface DrivingIO {
    public static class DriveData {
        double positionRad = 0;
        double positionReal = 0;
        double velocity = 0;
    }

    public default void updateData(DriveData data){
    };

    public default void setVoltage(double volts){
    };
}
