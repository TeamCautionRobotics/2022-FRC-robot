package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {
  
  private final SpeedControllerGroup leftDriveGroup;
  private final SpeedControllerGroup rightDriveGroup;

  private final Encoder leftEncoder;
  private final Encoder rightEncoder;

  private boolean usingLeftEncoder = true;


  public DriveBase(SpeedControllerGroup leftDriveGroup, SpeedControllerGroup rightDriveGroup, Encoder leftEncoder, Encoder rightEncoder) {  

    this.leftDriveGroup = leftDriveGroup;
    this.rightDriveGroup = rightDriveGroup;
    this.leftEncoder = leftEncoder;
    this.rightEncoder = rightEncoder;

  }

  public void drive(double leftPower, double rightPower) {
    this.leftDriveGroup.set(leftPower);
    this.rightDriveGroup.set(rightPower);
  }

  public void drive(double power) {
    this.leftDriveGroup.set(power);
    this.rightDriveGroup.set(power);
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void useLeftEncoder(boolean useLeft) {
    usingLeftEncoder = useLeft;
  }

  public double getDistance() {
    if (usingLeftEncoder) {
      return leftEncoder.getDistance();
    } else {
      return rightEncoder.getDistance();
    }
  }

  public double getRate() {
    if (usingLeftEncoder) {
      return leftEncoder.getRate();
    } else {
      return rightEncoder.getRate();
    }
  }

}
