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

    public static final class Climb {

        public static final class misc {

            public static final double calibrationSuccessDelay = 0.25;
            public static final double calibrationOverallFailDelay = 2.0 + calibrationSuccessDelay;

        }

        public static final class hook {

            public static final int hookPCMChannel = 0;

        }
        
        public static final class lift {

            // CAN IDS for motor controllers
            public static final int leftMotorID = 31;
            public static final int rightMotorID = 30;

            // DIO ports for switches
            public static final int leftSwitchPort = 0;
            public static final int rightSwitchPort = 1;

            // motor -> 3:1 -> 3:1 -> drum (r=0.7125in)
            public static final double encoderConversionFactor = (Math.PI * 1.425) * (1.0 / 9.0);  // output in inches

            public static final double initialReference = 0.0;
            public static final double kP = 0.17;
            public static final double kI = 0.06;
            public static final double kD = 0.002;
            public static final double kIMin = -0.3;
            public static final double kIMax = 0.5;

            // current thresholds
            public static final double maxCurrentThreshold = 65.0;  // failiure current threshold
            public static final double maxCurrentTimeout = 1.0;  // if the above current is exceeded for this long, fail

        }

        public static final class angle {

            // CAN IDs for motor controllers
            public static final int leftMotorID = 32;
            public static final int rightMotorID = 33;

            // DIO ports for switches
            public static final int leftSwitchPort = 2;
            public static final int rightSwitchPort = 3;

            public static final double initialSetpoint = 0.0;
            public static final double kP = 0.0115;
            public static final double kI = 0.0000001;
            public static final double kD = 0.00085;
            public static final double kIMax = 0.0425;
            public static final double kIMin = -0.0425;

            // math
            public static final double encoderConversionFactor = 360.0 / (4096.0 * 50.0 * 1.75);  // return units in degrees

        }

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