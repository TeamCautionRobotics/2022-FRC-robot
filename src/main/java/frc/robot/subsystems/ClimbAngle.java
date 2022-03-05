package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ClimbAngle extends SubsystemBase {

  private final WPI_TalonSRX leftAngleMotor;
  private final WPI_TalonSRX rightAngleMotor;

  private final DigitalInput leftArmAtZeroSwitch;
  private final DigitalInput rightArmAtZeroSwitch;

  // angler pid 
  private final PIDController leftAnglePID;
  private final PIDController rightAnglePID;
  private boolean anglePidEnabled = true;
  private boolean anglePidDisabled = false;
  private double angleSetpoint = Constants.Climb.angler.initialSetpoint;

  public ClimbAngle(
    WPI_TalonSRX leftAngleMotor, WPI_TalonSRX rightAngleMotor, 
    DigitalInput leftArmAtZeroSwitch, DigitalInput rightArmAtZeroSwitch) {

    this.leftAngleMotor = leftAngleMotor;
    leftAnglePID = new PIDController(Constants.Climb.angler.kP, Constants.Climb.angler.kI, Constants.Climb.angler.kD);

    this.rightAngleMotor = rightAngleMotor;
    rightAnglePID = new PIDController(Constants.Climb.angler.kP, Constants.Climb.angler.kI, Constants.Climb.angler.kD);

    // limit switches
    this.leftArmAtZeroSwitch = leftArmAtZeroSwitch;
    this.rightArmAtZeroSwitch = rightArmAtZeroSwitch;

  }

  /**
   * enable/disable the angle pid
   * @param enable
   */
  public void enableAnglePid(boolean enable) {
    anglePidEnabled = enable;
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
  public void setPosition(double reference) {
    angleSetpoint = reference;
  }

  /**
   * sets the angle motor power manually
   * the angle pid must be disabled for this to work
   * @param power the desired power output (-1.0 to 1.0)
   */
  public void setPower(double power) {
    leftAngleMotor.set(power);
    rightAngleMotor.set(power);
  }

  /**
   * sets the neutral mode of the angle controllers
   * @param mode
   */
  public void setNeutralMode(NeutralMode mode) {
    leftAngleMotor.setNeutralMode(mode);
    rightAngleMotor.setNeutralMode(mode);
  }

  /**
   * sets the coefficient to multiply the encoder readings by
   * @param coeff
   */
  public void setEncoderCoefficient(double coeff) {
    leftAngleMotor.configSelectedFeedbackCoefficient(coeff);
    rightAngleMotor.configSelectedFeedbackCoefficient(coeff);
  }

  /**
   * sets the angle encoder's position (useful for zeroing)
   * @param pos position to set
   */
  public void setEncoderPosition(double pos) {
    leftAngleMotor.setSelectedSensorPosition(pos);
    rightAngleMotor.setSelectedSensorPosition(pos);
  }

  /**
   * @return angle reference point
   */
  public double getSetpoint() {
    return angleSetpoint;
  }

  /**
   * @return the getDistance method of the left angle encode
   */
  public double getLeftEncoderDistance() {
    return leftAngleMotor.getSelectedSensorPosition();
  }

  /**
   * @return the getDistance method of the right angle encoder
   */
  public double getRightEncoderDistance() {
    return rightAngleMotor.getSelectedSensorPosition();
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

  /**
   * cut power to angle motors
   */
  public void stop() {
    leftAngleMotor.stopMotor();
    rightAngleMotor.stopMotor();
  }

  @Override
  public void periodic() {

    // update PID for angle
    if (anglePidEnabled) {
      anglePidDisabled = false;
      leftAngleMotor.set(leftAnglePID.calculate(getLeftEncoderDistance(), angleSetpoint));
      rightAngleMotor.set(rightAnglePID.calculate(getRightEncoderDistance(), angleSetpoint));
    } else {
      
      if (!anglePidDisabled) {
        anglePidDisabled = true;
        leftAngleMotor.set(0);
        rightAngleMotor.set(0);
      }

    }
    
  }

}
