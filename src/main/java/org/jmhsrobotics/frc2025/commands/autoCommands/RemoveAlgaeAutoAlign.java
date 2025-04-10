package org.jmhsrobotics.frc2025.commands.autoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.commands.autoAlign.AutoAlign;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.vision.Vision;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;
import org.littletonrobotics.junction.Logger;

public class RemoveAlgaeAutoAlign extends Command {
  private Drive drive;
  private Vision vision;
  private Elevator elevator;

  private int targetTagID;
  private double goalAngle;
  private Pose3d tagPose = new Pose3d();
  private Pose3d lastTagPose = null;

  private final PIDController xController = new PIDController(0.5, 0, 0.01);
  private final PIDController yController = new PIDController(0.5, 0, 0.01);
  private final PIDController thetaController = new PIDController(0.01, 0, 0);

  private final Transform2d algaeIntakeTransform = new Transform2d(0.29, 0, new Rotation2d());
  private final Transform2d algaeLineupTransform = new Transform2d(0.7, 0, new Rotation2d());
  private final Transform2d algaeInIntakeTransform = new Transform2d(1.0, 0, new Rotation2d());

  private Transform2d goalTransform = new Transform2d();

  public RemoveAlgaeAutoAlign(
      Drive drive, Vision vision, Elevator elevator, Wrist wrist, int targetTagID) {
    this.drive = drive;
    this.vision = vision;
    this.elevator = elevator;

    this.targetTagID = targetTagID;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    thetaController.reset();

    goalTransform = algaeLineupTransform;
    this.targetTagID = AutoAlign.adjustTagID(this.targetTagID);
    this.goalAngle = AutoAlign.calculateGoalAngleFromId(this.targetTagID);
  }

  @Override
  public void execute() {
    if (Math.abs(tagPose.getX() - goalTransform.getX()) < Units.inchesToMeters(5.5)
        && Math.abs(tagPose.getY() - goalTransform.getY()) < Units.inchesToMeters(5.5)
        && Math.abs(drive.getRotation().getDegrees() - goalAngle) < 3) {
      if (goalTransform == algaeLineupTransform) goalTransform = algaeIntakeTransform;
      else if (goalTransform == algaeIntakeTransform) goalTransform = algaeInIntakeTransform;
    }

    tagPose = AutoAlign.getTagPoseRobotRelative(targetTagID, vision, lastTagPose, drive.getPose());

    if (tagPose != null) {
      lastTagPose =
          new Pose3d(drive.getPose())
              .plus(new Transform3d(tagPose.getTranslation(), tagPose.getRotation()));
    }
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
                thetaController, this.goalAngle, drive.getRotation()));

    drive.runVelocity(outputSpeeds);

    Logger.recordOutput("Algae Intake/Target Tag Pose1", tagPose);
    Logger.recordOutput("Algae Intake/Target Tag ID1", this.targetTagID);
    Logger.recordOutput("Algae Intake/Target Angle", this.goalAngle);
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return Math.abs(tagPose.getX() - goalTransform.getX()) < Units.inchesToMeters(5.5)
        && Math.abs(tagPose.getY() - goalTransform.getY()) < Units.inchesToMeters(5.5)
        && Math.abs(drive.getRotation().getDegrees() - goalAngle) < 3
        && goalTransform == algaeInIntakeTransform;
  }
}
