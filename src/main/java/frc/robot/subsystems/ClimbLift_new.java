package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbLift_new extends SubsystemBase {

  // lifter objects
  private final CANSparkMax leftLifterMotor;
  private final RelativeEncoder leftLifterEncoder;
  private final CANSparkMax rightLifterMotor;
  private final RelativeEncoder rightLifterEncoder;

  // lifter pid objects
  private boolean PIDEnabled = true;
  private double lifterSetpoint = Constants.Climb.lift.initialReference;
  private ControlType lifterControlType = ControlType.kPosition;
  private double lifter_kP = Constants.Climb.lift.kP;
  private double lifter_kI = Constants.Climb.lift.kI;
  private double lifter_kD = Constants.Climb.lift.kD;

  // limit switches
  private DigitalInput leftArmFullyDownSwitch;
  private DigitalInput rightArmFullyDownSwitch;
  

  public ClimbLift_new(CANSparkMax leftLifter, CANSparkMax rightLifter, DigitalInput leftArmFullyDownSwitch, DigitalInput rightArmFullyDownSwitch) {

    // limit switches
    this.leftArmFullyDownSwitch = leftArmFullyDownSwitch;
    this.rightArmFullyDownSwitch = rightArmFullyDownSwitch;

    // lifter
    this.leftLifterMotor = leftLifter;
    leftLifterEncoder = leftLifter.getEncoder();
    this.rightLifterMotor = rightLifter;
    rightLifterEncoder = rightLifter.getEncoder();

    // set lifter conversion factor (divide by 60 to get /sec instead of /min)
    leftLifterEncoder.setPositionConversionFactor(Constants.Climb.lift.encoderConversionFactor);
    leftLifterEncoder.setVelocityConversionFactor(Constants.Climb.lift.encoderConversionFactor/60.0);
    rightLifterEncoder.setPositionConversionFactor(Constants.Climb.lift.encoderConversionFactor);
    rightLifterEncoder.setVelocityConversionFactor(Constants.Climb.lift.encoderConversionFactor/60.0);

  }

  /**
   * enable or disable the lift pid
   * @param e
   */
  public void enablePID(boolean e) {
    PIDEnabled = e;
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
  public void setPosition(double pos, ControlType controlType) {
    lifterSetpoint = pos;
    lifterControlType = controlType;
  }

  /**
   * manually set the power of the lift motors.
   * @param power the desired lifter motor power
   */
  public void setPower(double power) {
    leftLifterMotor.set(power);
    rightLifterMotor.set(power);
  }

  /**
   * @return lifter setpoint
   */
  public double getSetpoint() {
    return lifterSetpoint;
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
   * cut power to lifter motors
   */
  public void stop() {
    leftLifterMotor.stopMotor();
    rightLifterMotor.stopMotor();
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

  @Override
  public void periodic() {

  }

}
