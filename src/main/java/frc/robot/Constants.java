package frc.robot;

public final class Constants {

    public static final class DriveBase {

        public static final double wheelSize = 4.0;  // inches
        public static final double gearboxReductionFactor = 9.47;  // 1 / 9.47
        public static final int motorTemperatureWarnThreshold = 60;  // 60 deg C
        public static final int motorCurrentWarnThreshold = 100; //  TODO: Find this
        public static final int motorCurrentMaxThreshold = 120; //  TODO: Find this

        // TODO: find these
        public static final int leftSpark0ID = 0;
        public static final int leftSpark1ID = 1;
        public static final int rightSpark0ID = 2;
        public static final int rightSpark1ID = 3;
    }
}
