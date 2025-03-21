package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.led.LED;
import org.jmhsrobotics.frc2025.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class AlignReef extends Command {
  private final Drive drive;
  private final Vision vision;
  private final LED led;
  private final Elevator elevator;

  private final PIDController xController = new PIDController(0.6, 0, 0);
  private final PIDController yController = new PIDController(0.6, 0, 0);
  private final PIDController thetaController = new PIDController(0.01, 0, 0);
  private double xGoalMeters = 0.48;
  private double yGoalMeters = Units.inchesToMeters(-7.375);
  private double thetaGoalDegrees = 0; // Janky only work for one angle now

  private LEDPattern progressPattern;
  private double initialDistance = 2;
  private double currentDistance = 2;

  private int targetTagId;
  private Pose3d lastTagPose = null;
  private Transform2d goalTransform;
  private Pose3d tagPose;

  private double theta = 0;
  private double xdist = 0;
  private double ydist = 0;

  // boolean for if bot should align left or right
  private boolean isLeft = true;

  public AlignReef(Drive drive, Vision vision, LED led, Elevator elevator, boolean isLeft) {
    this.drive = drive;
    this.vision = vision;
    this.led = led;
    this.elevator = elevator;
    this.isLeft = isLeft;

    progressPattern =
        LEDPattern.progressMaskLayer(() -> ((initialDistance - currentDistance) / initialDistance));

    addRequirements(drive, led);
  }

  @Override
  public void initialize() {
    // Default setpoint: L4 left side(i think)
    xController.reset();
    yController.reset();
    thetaController.reset();

    // goal transform is recalculated in execute because the elevator height can change while it is
    // running
    this.goalTransform = AutoAlign.calculateReefTransform(this.elevator.getSetpoint(), isLeft);
    // angle goal and target ID do not ever need to be recalculated
    this.thetaGoalDegrees = AutoAlign.calculateGoalAngle(drive.getRotation().getDegrees());
    this.targetTagId = AutoAlign.calculateGoalTargetID(this.thetaGoalDegrees);

    xController.setSetpoint(xGoalMeters);
    yController.setSetpoint(yGoalMeters);
    double driveAngle = drive.getRotation().getDegrees();
    this.thetaGoalDegrees = AutoAlign.calculateGoalAngle(driveAngle);

    thetaController.setSetpoint(thetaGoalDegrees);
    thetaController.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    // New Reef Auto Align should have:
    // calculates goal transform
    this.goalTransform = AutoAlign.calculateReefTransform(this.elevator.getSetpoint(), isLeft);

    // gets target tag pose relative to bot
    tagPose =
        AutoAlign.getTagPoseRobotRelative(
            this.targetTagId, this.vision, this.lastTagPose, this.drive.getPose());

    if (tagPose != null) {
      lastTagPose =
          new Pose3d(drive.getPose())
              .plus(new Transform3d(tagPose.getTranslation(), tagPose.getRotation()));
      ChassisSpeeds outputSpeeds = new ChassisSpeeds();
      // calculate chassis speeds based on tag pose relative to bot and goal transformation from
      // that tag pose
      outputSpeeds =
          outputSpeeds.plus(
              AutoAlign.getReefAlignSpeeds(
                  tagPose, this.goalTransform, this.xController, this.yController));
      // calculate theta speeds
      outputSpeeds =
          outputSpeeds.plus(
              AutoAlign.getAutoAlignThetaSpeeds(
                  thetaController, this.thetaGoalDegrees, drive.getRotation()));

      drive.runVelocity(outputSpeeds);
      Logger.recordOutput("Align Reef/Target Tag Pose", tagPose);
      Logger.recordOutput("Align Reef/Target Tag ID", this.targetTagId);
    }
  }

  @Override
  public boolean isFinished() {
    // if (tagPose != null)
    //   return Math.abs(tagPose.getX() - goalTransform.getX()) < Units.inchesToMeters(1)
    //       && Math.abs(tagPose.getY() - goalTransform.getY()) < Units.inchesToMeters(1);
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
