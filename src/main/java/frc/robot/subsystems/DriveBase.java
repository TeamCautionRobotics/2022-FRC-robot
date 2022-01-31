package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;

public class DriveBase extends SubsystemBase {
  
  private final CANSparkMax leftSpark0;
  private final CANSparkMax leftSpark1;
  private final CANSparkMax rightSpark0;
  private final CANSparkMax rightSpark1;

  private final MotorControllerGroup leftDriveGroup;
  private final MotorControllerGroup rightSparkGroup;

  private final RelativeEncoder leftEncoder0;
  private final RelativeEncoder leftEncoder1;
  private final RelativeEncoder rightEncoder0;
  private final RelativeEncoder rightEncoder1;

  private boolean usingLeftEncoder = true;


  public DriveBase(CANSparkMax leftSpark0, CANSparkMax leftSpark1, CANSparkMax rightSpark0, CANSparkMax rightSpark1) {  

    this.leftSpark0 = leftSpark0;
    this.leftSpark1 = leftSpark1;
    this.rightSpark0 = rightSpark0;
    this.rightSpark1 = rightSpark1;

    this.leftDriveGroup = new MotorControllerGroup(leftSpark0, leftSpark1);
    this.rightSparkGroup = new MotorControllerGroup(rightSpark0, rightSpark1);

    this.leftEncoder0 = leftSpark0.getEncoder();
    this.leftEncoder1 = leftSpark1.getEncoder();
    this.rightEncoder0 = rightSpark0.getEncoder();
    this.rightEncoder1 = rightSpark1.getEncoder();
    
    // Invert left side
    this.leftSpark0.setInverted(true);
    this.leftSpark1.setInverted(true);

  }
  
  public void drive(double leftPower, double rightPower) {
    this.leftDriveGroup.set(leftPower);
    this.rightSparkGroup.set(rightPower);
  }

  public void drive(double power) {
    this.leftDriveGroup.set(power);
    this.rightSparkGroup.set(power);
  }

  public void resetEncoders() {
    leftEncoder0.setPosition(0);
    leftEncoder1.setPosition(0);
    rightEncoder0.setPosition(0);
    rightEncoder1.setPosition(0);
  }

  public void useLeftEncoder(boolean useLeft) {
    usingLeftEncoder = useLeft;
  }

  public double getDistance() {
    if (usingLeftEncoder) {
      return (leftEncoder0.getPosition() + leftEncoder1.getPosition()) / 2;
    } else {
      return (rightEncoder0.getPosition() + rightEncoder1.getPosition()) / 2;
    }
  }

  public double getDistanceDeviation() {
    if (usingLeftEncoder) {
      return leftEncoder0.getPosition() - leftEncoder1.getPosition();
    } else {
      return rightEncoder0.getPosition() - rightEncoder1.getPosition();
    }
  }

  public double getRate() {
    if (usingLeftEncoder) {
      return (leftEncoder0.getVelocity() + leftEncoder1.getVelocity()) / 2;
    } else {
      return (rightEncoder0.getVelocity() + rightEncoder1.getVelocity()) / 2;
    }
  }

  /**
   * Set the conversion factor for the encoders. Base unit is 1/revolution.
   *
   * @param distancePerPulse The conversion factor to multiply the native units by
   */
  public void setDistancePerPulse(double distancePerPulse) {
    leftEncoder0.setPositionConversionFactor(distancePerPulse);
    leftEncoder1.setPositionConversionFactor(distancePerPulse);
    rightEncoder0.setPositionConversionFactor(distancePerPulse);
    rightEncoder1.setPositionConversionFactor(distancePerPulse);

    // divide by 60 to make inches/min -> inches/sec
    leftEncoder0.setVelocityConversionFactor(distancePerPulse / 60.0);
    leftEncoder1.setVelocityConversionFactor(distancePerPulse / 60.0);
    rightEncoder0.setVelocityConversionFactor(distancePerPulse / 60.0);
    rightEncoder1.setVelocityConversionFactor(distancePerPulse / 60.0);
  }

  public double getMotorTemperature(int motor) {
    switch (motor) {
      case 0:
        return leftSpark0.getMotorTemperature();
      case 1:
        return leftSpark1.getMotorTemperature();
      case 2:
        return rightSpark0.getMotorTemperature();
      case 3:
        return rightSpark1.getMotorTemperature();
      default:
        throw new IndexOutOfBoundsException("motor selection was not 0-3!");
    }
  }

  public double getMotorCurrent(int motor) {
    switch (motor) {
      case 0:
        return leftSpark0.getOutputCurrent();
      case 1:
        return leftSpark1.getOutputCurrent();
      case 2:
        return rightSpark0.getOutputCurrent();
      case 3:
        return rightSpark1.getOutputCurrent();
      default:
        throw new IndexOutOfBoundsException("motor selection was not 0-3!");
    }
  }

  @Override
  public void periodic() {

    for (int n = 0; n < 4; n++) {

      // if (getMotorTemperature(n) > Constants.DriveBase.motorTemperatureWarnThreshold) {
      //   System.out.println(String.format("\n!!!!WARNING!!!! ---- Motor %d exceeded temperature!", n));
      // }

      if (getMotorCurrent(n) > Constants.DriveBase.motorCurrentWarnThreshold) {
        System.out.println(String.format("\n!!!!WARNING!!!! ---- Motor %d exceeded %d amps!", n, Constants.DriveBase.motorCurrentWarnThreshold));
      }

    }

  }

}
