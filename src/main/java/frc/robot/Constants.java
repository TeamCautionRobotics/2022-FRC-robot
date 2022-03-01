package frc.robot;

public final class Constants {

    public static final class Driver {

        public static final int leftJoystickPort = 0;
        public static final int rightJoystickPort = 1;
        public static final int manipulatorPort = 2;
        
    }

    public static final class Misc {

        public static final int pcmID = 1;

    }

    public static final class DriveBase {

        public static final double wheelSize = 4.0;  // inches
        public static final double gearboxReductionFactor = 9.47;  // 1 / 9.47
        public static final int motorTemperatureWarnThreshold = 60;  // 60 deg C
        public static final int motorCurrentWarnThreshold = 100; //  TODO: Find this
        public static final int motorCurrentMaxThreshold = 120; //  TODO: Find this

        // TODO: find these
        public static final int leftSpark0ID = 10;
        public static final int leftSpark1ID = 11;
        public static final int rightSpark0ID = 12;
        public static final int rightSpark1ID = 13;
    }

    public static final class Intake {

        // TODO: Find these
        public static final int pistonPCMChannel = 02;
        public static final int intakeMotorID = 20;

    }
}
