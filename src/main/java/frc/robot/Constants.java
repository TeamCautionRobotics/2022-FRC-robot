package frc.robot;

public final class Constants {

    public static final class Driver {

        public static final int leftJoystickPort = 0;
        public static final int rightJoystickPort = 1;
        
    }

    public static final class Misc {

        public static final int pcmID = 1;

    }

    public static final class DriveBase {

        public static final double encoderConversionFactor = (1.0 / 9.47) * 4.0 * Math.PI;
        public static final int motorTemperatureWarnThreshold = 60;
        public static final int motorCurrentWarnThreshold = 100;
        public static final int motorCurrentMaxThreshold = 120; 

        public static final int leftMotor0ID = 10;
        public static final int leftMotor1ID = 11;
        public static final int rightMotor0ID = 12;
        public static final int rightMotor1ID = 13;
    }

    public static final class Conveyor {

        public static final int pistonPCMChannel = 02;
        public static final int motorID = 21;
    }
        
    public static final class Intake {

        public static final int pistonPCMChannel = 01;
        public static final int motorID = 20;

    }
}
