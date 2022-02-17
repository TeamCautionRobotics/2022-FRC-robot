package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbLift extends SubsystemBase {

  // lifter objects
  private final CANSparkMax leftLifterMotor;
  private final RelativeEncoder leftLifterEncoder;
  private final SparkMaxPIDController leftLifterPID;
  private final CANSparkMax rightLifterMotor;
  private final RelativeEncoder rightLifterEncoder;
  private final SparkMaxPIDController rightLifterPID;

  // lifter pid objects
  private double lifterReference = Constants.Climb.lift.initialReference;
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

  public ClimbLift(CANSparkMax leftLifter, CANSparkMax rightLifter) {

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
   * set the desired lifter position
   * @param reference desired point (length in inches)
   */
  public void setReference(double reference) {
    lifterReference = reference;
  }

  /**
   * @return lifter reference point
   */
  double getReference() {
    return lifterReference;
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

    // check if pid vars have changed. if they have, update the relevant data in the controllers
    if (lifterReference != lifterReference_last) {
      leftLifterPID.setReference(lifterReference, ControlType.kPosition);
      rightLifterPID.setReference(lifterReference, ControlType.kPosition);
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