package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbAngle extends SubsystemBase {

  private final MotorController leftAngleMotor;
  private final MotorController rightAngleMotor;
  private final Encoder leftAngleEncoder;
  private final Encoder rightAngleEncoder;

  private final DigitalInput leftArmAtZeroSwitch;
  private final DigitalInput rightArmAtZeroSwitch;

  // angler pid 
  private final PIDController leftAnglePID;
  private final PIDController rightAnglePID;
  private boolean anglePidEnabled = true;
  private double angleReference = Constants.Climb.angler.initialSetpoint;

  public ClimbAngle(
    MotorController leftAngleMotor, MotorController rightAngleMotor, 
    Encoder leftAngleEncoder, Encoder rightAngleEncoder,
    DigitalInput leftArmAtZeroSwitch, DigitalInput rightArmAtZeroSwitch) {

    this.leftAngleMotor = leftAngleMotor;
    this.leftAngleEncoder = leftAngleEncoder;
    this.leftAngleEncoder.setDistancePerPulse(Constants.Climb.angler.encoderDistancePerPulse);
    leftAnglePID = new PIDController(Constants.Climb.angler.kP, Constants.Climb.angler.kI, Constants.Climb.angler.kD);

    this.rightAngleMotor = rightAngleMotor;
    this.rightAngleEncoder = rightAngleEncoder;
    this.rightAngleEncoder.setDistancePerPulse(Constants.Climb.angler.encoderDistancePerPulse);
    rightAnglePID = new PIDController(Constants.Climb.angler.kP, Constants.Climb.angler.kI, Constants.Climb.angler.kD);

    // limit switches
    this.leftArmAtZeroSwitch = leftArmAtZeroSwitch;
    this.rightArmAtZeroSwitch = rightArmAtZeroSwitch;

  }

  /**
   * updates the angle pid vars with new data.
   * THIS WILL BE LOST IF ROBOT IS POWERED OFF!
   * @param kP
   * @param kI
   * @param kD
   */
  public void setPidVars(double kP, double kI, double kD) {
    leftAnglePID.setP(kP);
    leftAnglePID.setI(kI);
    leftAnglePID.setD(kD);
    rightAnglePID.setP(kP);
    rightAnglePID.setI(kI);
    rightAnglePID.setD(kD);
  }

  /**
   * set the desired angle 
   * @param reference angle
   */
  public void setAngleReference(double reference) {
    angleReference = reference;
  }

  /**
   * sets the angle motor power manually
   * the angle pid must be disabled for this to work
   * @param power the desired power output (-1.0 to 1.0)
   */
  public void setAnglePower(double power) {
    leftAngleMotor.set(power);
    rightAngleMotor.set(power);
  }

  /**
   * @return angle reference point
   */
  public double getAngleReference() {
    return angleReference;
  }

  /**
   * @return the getDistance method of the left angle encoder
   */
  public double getLeftAngleEncoderDistance() {
    return leftAngleEncoder.getDistance();
  }

  /**
   * @return the getDistance method of the right angle encoder
   */
  public double getRightAngleEncoderDistance() {
    return rightAngleEncoder.getDistance();
  }

  /**
   * cut power to angle motors
   */
  public void stop() {
    leftAngleMotor.stopMotor();
    rightAngleMotor.stopMotor();
  }

  /**
   * enable/disable the angle pid
   * useful for stoppng all power output to the motors
   * @param enable
   */
  public void enableAnglePid(boolean enable) {
    anglePidEnabled = enable;
  }

  /**
   * get left arm angle switch (corrected for wiring of bot)
   */
  public boolean getLeftArmAtZeroSwitch() {
    return !leftArmAtZeroSwitch.get();
  }

  /**
   * get right arm angle switch (corrected for wiring of bot)
   */
  public boolean getRightArmAtZeroSwitch() {
    return !rightArmAtZeroSwitch.get();
  }

  @Override
  public void periodic() {

    // update PID for angle
    if (anglePidEnabled) {
      leftAngleMotor.set(leftAnglePID.calculate(leftAngleEncoder.getDistance(), angleReference));
      rightAngleMotor.set(rightAnglePID.calculate(rightAngleEncoder.getDistance(), angleReference));
    }
    
  }

}
