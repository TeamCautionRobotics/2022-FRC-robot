package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ClimbAngle extends SubsystemBase {

  private final WPI_TalonSRX leftMotor;
  private final WPI_TalonSRX rightMotor;

  private final DigitalInput leftArmAtZeroSwitch;
  private final DigitalInput rightArmAtZeroSwitch;

  // angler pid 
  private final PIDController leftAnglePID = new PIDController(Constants.Climb.angle.kP, Constants.Climb.angle.kI, Constants.Climb.angle.kD);
  private final PIDController rightAnglePID = new PIDController(Constants.Climb.angle.kP, Constants.Climb.angle.kI, Constants.Climb.angle.kD);
  private boolean pidEnabled = false;
  private boolean pidDisabled = false;
  private double setpoint = Constants.Climb.angle.initialSetpoint;

  private boolean calibrated = false;

  private double leftRampPrevOut = 0;
  private double leftRampOut = 0;
  private double rightRampPrevOut = 0;
  private double rightRampOut = 0;

  public ClimbAngle(
    WPI_TalonSRX leftAngleMotor, WPI_TalonSRX rightAngleMotor, 
    DigitalInput leftArmAtZeroSwitch, DigitalInput rightArmAtZeroSwitch) {

    setIRange(Constants.Climb.angle.kIMax, Constants.Climb.angle.kIMin);

    // motor controllers
    this.leftMotor = leftAngleMotor;
    this.rightMotor = rightAngleMotor;

    // limit switches
    this.leftArmAtZeroSwitch = leftArmAtZeroSwitch;
    this.rightArmAtZeroSwitch = rightArmAtZeroSwitch;

  }

  public void setCalibrated(boolean s) {
    this.calibrated = s;
  }
  
  public boolean getCalibrated() {
    return this.calibrated;
  }

  /**
   * enable/disable the angle pid
   * @param enable
   */
  public void enablePID(boolean enable) {
    pidEnabled = enable;
  }

  public void setP(double kP) {
    leftAnglePID.setP(kP);
    rightAnglePID.setP(kP);
  }

  public void setI(double kI) {
    leftAnglePID.setI(kI);
    rightAnglePID.setI(kI);
  }

  public void setD(double kD) {
    leftAnglePID.setD(kD);
    rightAnglePID.setD(kD);
  }

  public void setIRange(double low, double high) {
    leftAnglePID.setIntegratorRange(low, high);
    rightAnglePID.setIntegratorRange(low, high);
  }

  /**
   * set the desired angle 
   * @param reference angle
   */
  public void setPosition(double reference) {

    setpoint = reference;

    // if (reference > 110) {
    //   setpoint = 0;
    // } else if (reference < 0) {
    //   setpoint = 0;
    // } else {
    //   setpoint = reference;
    // }

  }

  /**
   * sets the angle motor power manually
   * the angle pid must be disabled for this to work
   * @param power the desired power output (-1.0 to 1.0)
   */
  public void setPower(double power) {
    leftMotor.set(power);
    rightMotor.set(power);
  }

  public void setPowerRamping(double leftPower, double rightPower, double rampBand) {

    if (Math.abs(leftPower - leftRampPrevOut) > rampBand) {
      leftRampOut = leftPower - leftRampPrevOut > 0 ? leftRampPrevOut + rampBand : leftRampPrevOut - rampBand;
    } else {
      leftRampOut = leftPower;
    }

    if (Math.abs(rightPower - rightRampPrevOut) > rampBand) {
      rightRampOut = rightPower - rightRampPrevOut > 0 ? rightRampPrevOut + rampBand : rightRampPrevOut - rampBand;
    } else {
      rightRampOut = rightPower;
    }

    leftMotor.set(leftRampOut);
    rightMotor.set(rightRampOut);
    leftRampPrevOut = leftRampOut;
    rightRampPrevOut = rightRampOut;

  }

  /**
   * sets the neutral mode of the angle controllers
   * @param mode
   */
  public void setNeutralMode(NeutralMode mode) {
    leftMotor.setNeutralMode(mode);
    rightMotor.setNeutralMode(mode);
  }
  
  /**
   * sets the angle encoder's position (useful for zeroing)
   * @param pos position to set
   */
  public void setEncoderPosition(double pos) {
    leftMotor.setSelectedSensorPosition(pos);
    rightMotor.setSelectedSensorPosition(pos-15);
  }

  /**
   * sets the coefficient to multiply the encoder readings by
   * @param coeff
   */
  public void setEncoderCoefficient(double coeff) {
    leftMotor.configSelectedFeedbackCoefficient(coeff);
    rightMotor.configSelectedFeedbackCoefficient(coeff);
  }

  /**
   * @return angle reference point
   */
  public double getSetpoint() {
    return setpoint;
  }

  /**
   * @return the getDistance method of the left angle encode
   */
  public double getLeftEncoderDistance() {
    return leftMotor.getSelectedSensorPosition();
  }

  /**
   * @return the getRate method of the left angle encoder
   */
  public double getLeftEncoderRate() {
    return leftMotor.getSelectedSensorVelocity();
  }

  /**
   * @return the getDistance method of the right angle encoder
   */
  public double getRightEncoderDistance() {
    return rightMotor.getSelectedSensorPosition();
  }

  /**
   * @return the getRate method of the left angle encoder
   */
  public double getRightEncoderRate() {
    return leftMotor.getSelectedSensorVelocity();
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

  public double getLeftPower() {
    return leftMotor.get();
  }

  public double getRightPower() {
    return rightMotor.get();
  }

  /**
   * cut power to angle motors
   */
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  @Override
  public void periodic() {

    // update PID for angle
    if (pidEnabled) {

      pidDisabled = false;
      // leftMotor.set(leftAnglePID.calculate(getLeftEncoderDistance(), setpoint));
      // rightMotor.set(rightAnglePID.calculate(getRightEncoderDistance(), setpoint));

      setPowerRamping(leftAnglePID.calculate(getLeftEncoderDistance(), setpoint),
                      rightAnglePID.calculate(getRightEncoderDistance(), setpoint),
                      Constants.Climb.angle.motorRampBand);

    } else {
      
      if (!pidDisabled) {
        pidDisabled = true;
        leftMotor.set(0);
        rightMotor.set(0);
      }

    }
    
  }

}
