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
        public static final int motorTemperatureWarnThreshold = 60;
        public static final int motorCurrentWarnThreshold = 100;
        public static final int motorCurrentMaxThreshold = 120;

        public static final int leftSpark0ID = 10;
        public static final int leftSpark1ID = 11;
        public static final int rightSpark0ID = 12;
        public static final int rightSpark1ID = 13;
    }

    public static final class Climb {

        public static final class hook {

            public static final int hookPCMChannel = 0;

        }
        
        public static final class lift {

            // CAN IDS for motor controllers
            public static final int leftMotorID = 31;
            public static final int rightMotorID = 30;

            // DIO ports for switches
            public static final int leftArmFullyDownSwitchPort = 0;
            public static final int rightArmFullyDownSwtichPort = 1;

            // motor -> 3:1 -> 3:1 -> drum (r=0.7125in)
            public static final double encoderConversionFactor = (Math.PI * 1.425) * (1.0 / 9.0);  // output in inches

            public static final double initialReference = 0.0;
            public static final double kP = 0.12;
            public static final double kI = 0.00001;
            public static final double kD = 0.002;
            public static final double kIMin = -0.1;
            public static final double kIMax = 0.1;

            // thresholds in inches
            public static final double fullyLoweredThreshold = 5.0;
            public static final double fullyRaisedThreshold = 50.0;

            // other misc constants
            public static final double safeModeLoweringVelocity = -0.1;

            // current thresholds
            public static final double maxCurrentThreshold = 100.0;  // if current exceeds this, the climb fails!

        }

        public static final class angler {

            // CAN IDs for motor controllers
            public static final int leftID = 32;
            public static final int rightID = 33;

            // DIO ports for switches
            public static final int leftArmAtZeroSwitchPort = 2;
            public static final int rightArmAtZeroSwitchPort = 3;

            // internal wpilib pid
            // TODO: find these
            public static final double initialSetpoint = 0.0;
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kIMax = 0.1;
            public static final double kIMin = -0.1;

            // math
            public static final double encoderDistancePerPulse = 360.0 / (4096.0 * 50.0 * 1.75);  // return units in degrees

        }

    }

    public static final class Conveyor {

        public static final int motorID = 21;
        public static final int gatePCMChannel = 02;

    }
        
    public static final class Intake {

        public static final int pistonPCMChannel = 01;
        public static final int intakeMotorID = 20;

    }
}