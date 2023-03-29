package frc.robot;

public class Ports {
    public interface motorPorts{
        int frontLeftDrive = 1;
        int frontLeftTurn = 2;

        int frontRightDrive = 3;
        int frontRightTurn = 4;

        int backLeftDrive = 5;
        int backLeftTurn = 6;

        int backRightDrive = 7;
        int backRightTurn = 8;
    }
    public interface encoderPorts{
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;
    }
}
