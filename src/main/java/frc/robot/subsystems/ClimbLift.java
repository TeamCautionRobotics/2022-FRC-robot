package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ClimbLift extends SubsystemBase {

  // lifter devices
  private final CANSparkMax leftLifterMotor;
  private final CANSparkMax rightLifterMotor;
  private final RelativeEncoder leftLifterEncoder;
  private final RelativeEncoder rightLifterEncoder;
  private DigitalInput leftArmFullyDownSwitch;
  private DigitalInput rightArmFullyDownSwitch;

  // lifter pid variables
  private PIDController leftPID = new PIDController(Constants.Climb.lift.kP, Constants.Climb.lift.kI, Constants.Climb.lift.kD);
  private PIDController rightPID = new PIDController(Constants.Climb.lift.kP, Constants.Climb.lift.kI, Constants.Climb.lift.kD);

  private boolean PIDEnabled = false;
  private boolean PIDDisabled = false;
  private double setpoint = Constants.Climb.lift.initialReference;

  private boolean calibrated = false;


  public ClimbLift(CANSparkMax leftLifter, CANSparkMax rightLifter, DigitalInput leftArmFullyDownSwitch, DigitalInput rightArmFullyDownSwitch) {

    this.setIRange(Constants.Climb.lift.kIMin, Constants.Climb.lift.kIMax);

    // lifter
    this.leftLifterMotor = leftLifter;
    leftLifterEncoder = leftLifter.getEncoder();
    this.rightLifterMotor = rightLifter;
    rightLifterEncoder = rightLifter.getEncoder();
    
    // limit switches
    this.leftArmFullyDownSwitch = leftArmFullyDownSwitch;
    this.rightArmFullyDownSwitch = rightArmFullyDownSwitch;

    // set lifter conversion factor (divide by 60 to get /sec instead of /min)
    leftLifterEncoder.setPositionConversionFactor(Constants.Climb.lift.encoderConversionFactor);
    leftLifterEncoder.setVelocityConversionFactor(Constants.Climb.lift.encoderConversionFactor/60.0);
    rightLifterEncoder.setPositionConversionFactor(Constants.Climb.lift.encoderConversionFactor);
    rightLifterEncoder.setVelocityConversionFactor(Constants.Climb.lift.encoderConversionFactor/60.0);

  }

  public void setCalibrated(boolean s) {
    this.calibrated = s;
  }

  public boolean getCalibrated() {
    return this.calibrated;
  }

  /**
   * enable or disable the lift pid
   * @param e
   */
  public void enablePID(boolean e) {
    PIDEnabled = e;
  }

  public void setP(double kP) {
    leftPID.setP(kP);
    rightPID.setP(kP);
  }

  public void setI(double kI) {
    leftPID.setI(kI);
    rightPID.setI(kI);
  }

  public void setD(double kD) {
    leftPID.setD(kD);
    rightPID.setD(kD);
  }

  public void setIRange(double low, double high) {
    leftPID.setIntegratorRange(low, high);
    rightPID.setIntegratorRange(low, high);
  }

  /**
   * sets the position of the encoders (useful for reset)
   * @param pos position to set
   */
  public void setEncoderPosition(double pos) {
    leftLifterEncoder.setPosition(pos);
    rightLifterEncoder.setPosition(pos);
  }

  /**
   * set the desired lifter position
   * @param pos desired point (length in inches)
   * @param controlType reference type
   */
  public void setPosition(double pos) {
    this.setpoint = pos;
  }

  /**
   * manually set the power of the lift motors.
   * 
   * <p> you must disable the pid to use this
   * @param power the desired lifter motor power
   */
  public void setPower(double power) {
    if (!PIDEnabled) {
      leftLifterMotor.set(power);
      rightLifterMotor.set(power);
    }
  }

  /**
   * sets the idle mode of the spark maxes
   * @param m
   */
  public void setIdleMode(IdleMode m) {
    leftLifterMotor.setIdleMode(m);
    rightLifterMotor.setIdleMode(m);
  }

  /**
   * @return lifter setpoint
   */
  public double getSetpoint() {
    return setpoint;
  }

  /**
   * @return the getPosition() method of the left lifter encoder
   */
  public double getLeftLiftPosition() {
    return leftLifterEncoder.getPosition();
  }

  /**
   * @return the getPosition() method of the right lifter encoder
   */
  public double getRightLiftPosition() {
    return rightLifterEncoder.getPosition();
  }

  /**
   * @return the output current of the left motor in amps
   */
  public double getLeftMotorCurrent() {
    return leftLifterMotor.getOutputCurrent();
  }

  /**
   * @return the output current of the right motor in amps
   */
  public double getRightMotorCurrent() {
    return rightLifterMotor.getOutputCurrent();
  }

  /**
   * get left switch state (true is pressed)
   */
  public boolean getLeftArmFullyDownSwitch() {
    return !leftArmFullyDownSwitch.get();
  }

  /**
   * get right switch state (true is pressed)
   */
  public boolean getRightArmFullyDownSwitch() {
    return !rightArmFullyDownSwitch.get();
  }

  /**
   * cut power to lifter motors
   */
  public void stop() {
    leftLifterMotor.stopMotor();
    rightLifterMotor.stopMotor();
  }

  @Override
  public void periodic() {

    if (PIDEnabled) {

      leftLifterMotor.set(leftPID.calculate(getLeftLiftPosition(), setpoint));
      rightLifterMotor.set(rightPID.calculate(getRightLiftPosition(), setpoint));

    } else {

      if (!PIDDisabled) {
        leftLifterMotor.set(0);
        rightLifterMotor.set(0);
        PIDDisabled = true;
      }

    }

  }

}
