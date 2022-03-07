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
  private final PIDController leftAnglePID = new PIDController(Constants.Climb.angler.kP, Constants.Climb.angler.kI, Constants.Climb.angler.kD);
  private final PIDController rightAnglePID = new PIDController(Constants.Climb.angler.kP, Constants.Climb.angler.kI, Constants.Climb.angler.kD);
  private boolean pidEnabled = false;
  private boolean pidDisabled = false;
  private double setpoint = Constants.Climb.angler.initialSetpoint;

  public ClimbAngle(
    WPI_TalonSRX leftAngleMotor, WPI_TalonSRX rightAngleMotor, 
    DigitalInput leftArmAtZeroSwitch, DigitalInput rightArmAtZeroSwitch) {

    setIRange(Constants.Climb.angler.kIMax, Constants.Climb.angler.kIMin);

    // motor controllers
    this.leftMotor = leftAngleMotor;
    this.rightMotor = rightAngleMotor;

    // limit switches
    this.leftArmAtZeroSwitch = leftArmAtZeroSwitch;
    this.rightArmAtZeroSwitch = rightArmAtZeroSwitch;

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

    if (reference > 100) {
      setpoint = 0;
    } else if (reference < 0) {
      setpoint = 0;
    } else {
      setpoint = reference;
    }
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
    rightMotor.setSelectedSensorPosition(pos);
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
   * @return the getDistance method of the right angle encoder
   */
  public double getRightEncoderDistance() {
    return rightMotor.getSelectedSensorPosition();
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
      leftMotor.set(leftAnglePID.calculate(getLeftEncoderDistance(), setpoint));
      rightMotor.set(rightAnglePID.calculate(getRightEncoderDistance(), setpoint));

    } else {
      
      if (!pidDisabled) {
        pidDisabled = true;
        leftMotor.set(0);
        rightMotor.set(0);
      }

    }
    
  }

}
