package org.jmhsrobotics.frc2025.commands.autoAlign;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.led.LED;
import org.jmhsrobotics.frc2025.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class AlignReefSetAngle extends Command {
  private final Drive drive;
  private final Vision vision;
  private final LED led;
  private final Elevator elevator;

  private final PIDController xController = new PIDController(0.3, 0, 0.005);
  private final PIDController yController = new PIDController(0.3, 0, 0.005);
  private final PIDController thetaController = new PIDController(0.01, 0, 0);

  private double thetaGoalDegrees = 0; // Janky only work for one angle now

  private LEDPattern progressPattern;
  private double initialDistance = 3;
  private double currentDistance = 3;

  private int targetTagId;
  private Pose3d lastTagPose = null;
  private Transform2d goalTransform;
  private Pose3d tagPose;

  // boolean for if bot should align left or right
  private boolean isLeft = true;

  public AlignReefSetAngle(
      Drive drive, Vision vision, LED led, Elevator elevator, boolean isLeft, int targetTagId) {
    this.drive = drive;
    this.vision = vision;
    this.led = led;
    this.elevator = elevator;
    this.isLeft = isLeft;
    this.targetTagId = targetTagId;

    progressPattern =
        LEDPattern.progressMaskLayer(
            () -> ((this.initialDistance - this.currentDistance) / this.initialDistance));

    addRequirements(drive, led);
  }

  private void adjustTagId() {
    if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
      switch (this.targetTagId) {
        case 17:
          this.targetTagId = 8;
          break;
        case 18:
          this.targetTagId = 7;
          break;
        case 19:
          this.targetTagId = 6;
          break;
        case 20:
          this.targetTagId = 11;
          break;
        case 21:
          this.targetTagId = 10;
          break;
        case 22:
          this.targetTagId = 9;
          break;
      }
    }
  }

  private double calculateGoalAngleFromId(int targetTagId) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      if (targetTagId == 18) return 0;
      else if (targetTagId == 17) return 60;
      else if (targetTagId == 22) return 120;
      else if (targetTagId == 21) return 180;
      else if (targetTagId == 19) return -60;
      else return -120;
    }

    // if current alliance is red, use the following angles
    if (targetTagId == 7) return 180;
    else if (targetTagId == 8) return -120;
    else if (targetTagId == 10) return 0;
    else if (targetTagId == 6) return 120;
    else if (targetTagId == 9) return -60;
    else return 60;
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    thetaController.reset();
    thetaController.enableContinuousInput(-180, 180);

    this.goalTransform = AutoAlign.calculateReefTransform(this.elevator.getSetpoint(), isLeft);
    adjustTagId();
    this.thetaGoalDegrees = calculateGoalAngleFromId(this.targetTagId);
  }

  @Override
  public void execute() {
    // New Reef Auto Align should have:
    // calculates goal transform. recalculated in execute because the elevator height can change
    // while it is running[\]

    this.goalTransform = AutoAlign.calculateReefTransform(this.elevator.getSetpoint(), isLeft);

    // gets target tag pose relative to bot
    tagPose =
        AutoAlign.getTagPoseRobotRelative(
            this.targetTagId, this.vision, this.lastTagPose, this.drive.getPose());

    if (tagPose != null) {
      lastTagPose =
          new Pose3d(drive.getPose())
              .plus(new Transform3d(tagPose.getTranslation(), tagPose.getRotation()));
      // calculate chassis speeds based on tag pose relative to bot and goal transformation from
      // that tag pose
      ChassisSpeeds outputSpeeds = new ChassisSpeeds();
      outputSpeeds =
          AutoAlign.getReefAlignSpeeds(
              tagPose, this.goalTransform, this.xController, this.yController);
      // calculate theta speeds
      outputSpeeds =
          outputSpeeds.plus(
              AutoAlign.getAutoAlignThetaSpeeds(
                  thetaController, this.thetaGoalDegrees, drive.getRotation()));

      drive.runVelocity(outputSpeeds);
      Logger.recordOutput("Align Reef/Target Tag Pose1", tagPose);
      Logger.recordOutput("Align Reef/Target Tag ID1", this.targetTagId);
      Logger.recordOutput("Align Reef/Target Angle1", this.thetaGoalDegrees);

      // calculates the distance from target for the LED progress pattern
      this.currentDistance =
          Math.sqrt(
              Math.pow(tagPose.getX() - goalTransform.getX(), 2)
                  + Math.pow(tagPose.getY() - goalTransform.getY(), 2));
      led.setPattern(progressPattern);
    }
  }

  @Override
  public boolean isFinished() {
    if (tagPose != null)
      return Math.abs(tagPose.getX() - goalTransform.getX()) < Units.inchesToMeters(1)
          && Math.abs(tagPose.getY() - goalTransform.getY()) < Units.inchesToMeters(1);
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
