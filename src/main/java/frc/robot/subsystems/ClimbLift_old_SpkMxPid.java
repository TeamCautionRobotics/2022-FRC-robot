package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbLift_old_SpkMxPid extends SubsystemBase {

  // lifter objects
  private final CANSparkMax leftLifterMotor;
  private final RelativeEncoder leftLifterEncoder;
  private final SparkMaxPIDController leftLifterPID;
  private final CANSparkMax rightLifterMotor;
  private final RelativeEncoder rightLifterEncoder;
  private final SparkMaxPIDController rightLifterPID;

  // lifter pid objects
  private double lifterReference = Constants.Climb.lift.initialReference;
  private ControlType lifterControlType = ControlType.kPosition;
  private double lifter_kP = Constants.Climb.lift.kP;
  private double lifter_kI = Constants.Climb.lift.kI;
  private double lifter_kD = Constants.Climb.lift.kD;
  private double lifter_iZ = Constants.Climb.lift.kIz;
  private double lifter_ff = Constants.Climb.lift.kFF;
  private double lifter_maxOutput = Constants.Climb.lift.kMaxOutput;
  private double lifter_minOutput = Constants.Climb.lift.kMinOutput;

  private double lifterReference_last = Constants.Climb.lift.initialReference;
  private double lifter_kP_last = Constants.Climb.lift.kP;
  private double lifter_kI_last = Constants.Climb.lift.kI;
  private double lifter_kD_last = Constants.Climb.lift.kD;
  private double lifter_iZ_last = Constants.Climb.lift.kIz;
  private double lifter_ff_last = Constants.Climb.lift.kFF;
  private double lifter_maxOutput_last = Constants.Climb.lift.kMaxOutput;
  private double lifter_minOutput_last = Constants.Climb.lift.kMinOutput;

  // limit switches
  private DigitalInput leftArmFullyDownSwitch;
  private DigitalInput rightArmFullyDownSwitch;
  

  public ClimbLift_old_SpkMxPid(CANSparkMax leftLifter, CANSparkMax rightLifter, DigitalInput leftArmFullyDownSwitch, DigitalInput rightArmFullyDownSwitch) {

    // limit switches
    this.leftArmFullyDownSwitch = leftArmFullyDownSwitch;
    this.rightArmFullyDownSwitch = rightArmFullyDownSwitch;

    // lifter
    this.leftLifterMotor = leftLifter;
    leftLifterEncoder = leftLifter.getEncoder();
    leftLifterPID = leftLifter.getPIDController();
    this.rightLifterMotor = rightLifter;
    rightLifterEncoder = rightLifter.getEncoder();
    rightLifterPID = rightLifter.getPIDController();

    // set lifter PID vars
    leftLifterPID.setP(lifter_kP);
    leftLifterPID.setI(lifter_kI);
    leftLifterPID.setD(lifter_kD);
    leftLifterPID.setIZone(lifter_iZ);
    leftLifterPID.setFF(lifter_ff);
    leftLifterPID.setOutputRange(lifter_minOutput, lifter_maxOutput);

    rightLifterPID.setP(lifter_kP);
    rightLifterPID.setI(lifter_kI);
    rightLifterPID.setD(lifter_kD);
    rightLifterPID.setIZone(lifter_iZ);
    rightLifterPID.setFF(lifter_ff);
    rightLifterPID.setOutputRange(lifter_minOutput, lifter_maxOutput);

    // set lifter conversion factor (divide by 60 to get /sec instead of /min)
    leftLifterEncoder.setPositionConversionFactor(Constants.Climb.lift.encoderConversionFactor);
    leftLifterEncoder.setVelocityConversionFactor(Constants.Climb.lift.encoderConversionFactor/60.0);
    rightLifterEncoder.setPositionConversionFactor(Constants.Climb.lift.encoderConversionFactor);
    rightLifterEncoder.setVelocityConversionFactor(Constants.Climb.lift.encoderConversionFactor/60.0);

  }

  /**
   * writes new pid vars to the spark maxes
   * @param kP
   * @param kI
   * @param kD
   * @param kIz
   * @param kFF
   * @param kOutMin
   * @param kOutMax
   */
  public void setPidVars(double kP, double kI, double kD, double kIz, double kFF, double kOutMin, double kOutMax) {
    lifter_kP = kP;
    lifter_kI = kI;
    lifter_kD = kD;
    lifter_iZ = kIz;
    lifter_ff = kFF;
    lifter_minOutput = kOutMin;
    lifter_maxOutput = kOutMax;
  }

    /**
   * writes new pid vars to the spark maxes
   * @param kP
   * @param kI
   * @param kD
   * @param kIz
   * @param kFF
   */
  public void setPidVars(double kP, double kI, double kD, double kIz, double kFF) {
    lifter_kP = kP;
    lifter_kI = kI;
    lifter_kD = kD;
    lifter_iZ = kIz;
    lifter_ff = kFF;
  }

  /**
   * set the desired lifter position
   * @param reference desired point (length in inches)
   * @param controlType reference type
   */
  public void setReference(double reference, ControlType controlType) {
    lifterReference = reference;
    lifterControlType = controlType;
  }

  /**
   * manually set the power of the lift motors.
   * this has an unknown effect on the internal spark max pid
   * i don't know if it will cancel it
   * @param power the desired lifter motor power
   */
  public void setPower(double power) {
    leftLifterMotor.set(power);
    rightLifterMotor.set(power);
  }

  /**
   * @return lifter reference point
   */
  public double getReference() {
    return lifterReference;
  }

  /**
   * @return the getPosition() method of the left lifter encoder
   */
  public double getLeftLiftPosition() {
    return leftLifterEncoder.getPosition();
  }

  /**
   * @return the getVelocity() method of the left lifter encoder
   */
  public double getLeftLiftVelocity() {
    return leftLifterEncoder.getVelocity();
  }

  /**
   * @return the getPosition() method of the right lifter encoder
   */
  public double getRightLiftPosition() {
    return rightLifterEncoder.getPosition();
  }

  /**
   * @return the getVelocity() method of the right lifter encoder
   */
  public double getRightLiftVelocity() {
    return rightLifterEncoder.getVelocity();
  }

  public void setLiftPosition(double pos) {
    leftLifterEncoder.setPosition(pos);
    rightLifterEncoder.setPosition(pos);
  }

  /**
   * returns the current consumption of a motor
   * @param motor 1: left, 2: right
   * @return the current consumption of the specified motor
   */
  public double getMotorCurrent(int motor) {

    double current = 0.0;

    if (motor == 0) {
      current = leftLifterMotor.getOutputCurrent();
    } else if (motor == 1) {
      current = rightLifterMotor.getOutputCurrent();
    } else {
      throw new IndexOutOfBoundsException();
    }

    return current;

  }

  /**
   * cut power to lifter motors
   */
  public void stop() {
    leftLifterMotor.stopMotor();
    rightLifterMotor.stopMotor();
  }

  /**
   * get left switch state (reversed to account for bot wiring)
   */
  public boolean getLeftArmFullyDownSwitch() {
    return !leftArmFullyDownSwitch.get();
  }

  /**
   * get right switch state (reversed to account for bot wiring)
   */
  public boolean getRightArmFullyDownSwitch() {
    return !rightArmFullyDownSwitch.get();
  }

  @Override
  public void periodic() {

    // check if pid vars have changed. if they have, update the relevant data in the controllers
    if (lifterReference != lifterReference_last) {
      leftLifterPID.setReference(lifterReference, lifterControlType);
      rightLifterPID.setReference(lifterReference, lifterControlType);
      lifterReference_last = lifterReference;
    }

    if (lifter_kP != lifter_kP_last) { leftLifterPID.setP(lifter_kP); rightLifterPID.setP(lifter_kP); lifter_kP_last = lifter_kP; }
    if (lifter_kI != lifter_kI_last) { leftLifterPID.setP(lifter_kI); rightLifterPID.setP(lifter_kI); lifter_kI_last = lifter_kI; }
    if (lifter_kD != lifter_kD_last) { leftLifterPID.setP(lifter_kD); rightLifterPID.setP(lifter_kD); lifter_kD_last = lifter_kD; }
    if (lifter_iZ != lifter_iZ_last) { leftLifterPID.setP(lifter_iZ); rightLifterPID.setP(lifter_iZ); lifter_iZ_last = lifter_iZ; }
    if (lifter_ff != lifter_ff_last) { leftLifterPID.setP(lifter_ff); rightLifterPID.setP(lifter_ff); lifter_ff_last = lifter_ff; }
    if ((lifter_minOutput != lifter_minOutput_last) || (lifter_maxOutput != lifter_maxOutput_last)) {
      leftLifterPID.setOutputRange(lifter_minOutput, lifter_maxOutput);
      lifter_minOutput_last = lifter_minOutput;
      lifter_maxOutput_last = lifter_maxOutput;
    }
    
  }

}
