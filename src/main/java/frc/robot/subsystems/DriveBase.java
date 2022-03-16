package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveBase extends SubsystemBase {
  
  private final CANSparkMax leftController0;
  private final CANSparkMax leftController1;
  private final CANSparkMax rightController0;
  private final CANSparkMax rightController1;

  private final MotorControllerGroup leftDriveGroup;
  private final MotorControllerGroup rightDriveGroup;

  private final RelativeEncoder leftEncoder0;
  private final RelativeEncoder leftEncoder1;
  private final RelativeEncoder rightEncoder0;
  private final RelativeEncoder rightEncoder1;

  private boolean usingLeftEncoder = true;

  private double leftRampPrevOut = 0;
  private double leftRampOut = 0;
  private double rightRampPrevOut = 0;
  private double rightRampOut = 0;


  public DriveBase(CANSparkMax leftController0, CANSparkMax leftController1, CANSparkMax rightController0, CANSparkMax rightController1) {  

    this.leftController0 = leftController0;
    this.leftController1 = leftController1;
    this.rightController0 = rightController0;
    this.rightController1 = rightController1;

    this.leftDriveGroup = new MotorControllerGroup(leftController0, leftController1);
    this.rightDriveGroup = new MotorControllerGroup(rightController0, rightController1);

    this.leftEncoder0 = leftController0.getEncoder();
    this.leftEncoder1 = leftController1.getEncoder();
    this.rightEncoder0 = rightController0.getEncoder();
    this.rightEncoder1 = rightController1.getEncoder();

  }

  public void setIdleMode(IdleMode m) {
    leftController0.setIdleMode(m);
    leftController1.setIdleMode(m);
    rightController0.setIdleMode(m);
    rightController1.setIdleMode(m);
  }
  
  public void drive(double leftPower, double rightPower) {
    this.leftDriveGroup.set(leftPower);
    this.rightDriveGroup.set(rightPower);
  }

  public void drive(double power) {
    this.leftDriveGroup.set(power);
    this.rightDriveGroup.set(power);
  }

  public void driveRamping(double leftPower, double rightPower, double rampBand) {

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

    this.leftDriveGroup.set(leftRampOut);
    this.rightDriveGroup.set(rightRampOut);
    leftRampPrevOut = leftRampOut;
    rightRampPrevOut = rightRampOut;

  }

  public void setEncoderDistance(double d) {
    leftEncoder0.setPosition(d);
    leftEncoder1.setPosition(d);
    rightEncoder0.setPosition(d);
    rightEncoder1.setPosition(d);
  }

  public void useLeftEncoder(boolean useLeft) {
    usingLeftEncoder = useLeft;
  }

  public double getDistance() {
    if (usingLeftEncoder) {
      return -(leftEncoder0.getPosition() + leftEncoder1.getPosition()) / 2;
    } else {
      return -(rightEncoder0.getPosition() + rightEncoder1.getPosition()) / 2;
    }
  }

  public double getDistance(int e) {
    if (e == 0) {
      return -(leftEncoder0.getPosition() + leftEncoder1.getPosition()) / 2;
    } else if (e == 1) {
      return -(rightEncoder0.getPosition() + rightEncoder1.getPosition()) / 2;
    } else {
      throw new IndexOutOfBoundsException();
    }
  }

  public double getRate() {
    if (usingLeftEncoder) {
      return -(leftEncoder0.getVelocity() + leftEncoder1.getVelocity()) / 2;
    } else {
      return -(rightEncoder0.getVelocity() + rightEncoder1.getVelocity()) / 2;
    }
  }

  public double getRate(int e) {
    if (e == 0) {
      return -(leftEncoder0.getVelocity() + leftEncoder1.getVelocity()) / 2;
    } else if (e == 1) {
      return -(rightEncoder0.getVelocity() + rightEncoder1.getVelocity()) / 2;
    } else {
      throw new IndexOutOfBoundsException();
    }
  }

  /**
   * Set the conversion factor for the encoders. Base unit is 1/revolution.
   *
   * @param distancePerPulse The conversion factor to multiply the native units by
   */
  public void setEncoderConversionFactor(double distancePerPulse) {
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
        return leftController0.getMotorTemperature();
      case 1:
        return leftController1.getMotorTemperature();
      case 2:
        return rightController0.getMotorTemperature();
      case 3:
        return rightController1.getMotorTemperature();
      default:
        throw new IndexOutOfBoundsException("motor selection was not 0-3!");
    }
  }

  public double getMotorCurrent(int motor) {
    switch (motor) {
      case 0:
        return leftController0.getOutputCurrent();
      case 1:
        return leftController1.getOutputCurrent();
      case 2:
        return rightController0.getOutputCurrent();
      case 3:
        return rightController1.getOutputCurrent();
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
