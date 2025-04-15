package org.jmhsrobotics.frc2025.commands.autoAlign;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.vision.VisionConstants;
import org.littletonrobotics.junction.Logger;

public class AlignSource extends Command {
  private Drive drive;
  private Pose2d goalPose;
  private boolean alignCloseToStation;

  private final PIDController xController = new PIDController(0.7, 0, 0.005);
  private final PIDController yController = new PIDController(0.7, 0, 0.005);
  private final PIDController thetaController = new PIDController(0.01, 0, 0);

  private final Timer timer = new Timer();

  public AlignSource(Drive drive, boolean alignCloseToStation) {
    this.drive = drive;
    this.alignCloseToStation = alignCloseToStation;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    this.goalPose = calculateSetpoints(drive, alignCloseToStation);
    xController.reset();
    yController.reset();
    thetaController.reset();
    thetaController.enableContinuousInput(-180, 180);

    xController.setSetpoint(this.goalPose.getX());
    yController.setSetpoint(this.goalPose.getY());
    thetaController.setSetpoint(this.goalPose.getRotation().getDegrees());

    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    this.goalPose = calculateSetpoints(drive, alignCloseToStation);
    drive.runVelocity(
        AutoAlign.getDriveToPoseSpeeds(
            this.drive, this.goalPose, this.xController, this.yController, this.thetaController));
    // calculateSourceAutoAlignSpeeds(
    //     this.drive, this.goalPose, this.xController, this.yController, this.thetaController));
    // Logger.recordOutput("Align Source/Goal Pose", this.goalPose);
  }

  @Override
  public boolean isFinished() {
    Transform2d distance = drive.getPose().minus(this.goalPose);
    return distance.getX() < Units.inchesToMeters(1)
        && distance.getY() < Units.inchesToMeters(1)
        && distance.getRotation().getDegrees() < 3;
    // return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    Logger.recordOutput("Align Source/Non Profiled Align Time", timer.get());
  }

  /**
   * calculates the coordinate setpoints for source align based drivetrain position and if the auto
   * align should be on the end further or closer to the drivers
   *
   * @param drive
   * @param alignCloseToSource
   * @return Pose2d
   */
  public static Pose2d calculateSetpoints(Drive drive, boolean alignCloseToSource) {
    double ySetpoint = 0.5;
    Pose2d targetTagPose;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      if (drive.getPose().getY() > 4) {
        targetTagPose =
            VisionConstants.aprilTagLayout.getTagPose(2).orElse(new Pose3d()).toPose2d();
        if (!alignCloseToSource) ySetpoint = -0.5;
      } else {
        targetTagPose =
            VisionConstants.aprilTagLayout.getTagPose(1).orElse(new Pose3d()).toPose2d();
        if (alignCloseToSource) ySetpoint = -0.5;
      }
    } else {
      if (drive.getPose().getY() > 4) {
        targetTagPose =
            VisionConstants.aprilTagLayout.getTagPose(13).orElse(new Pose3d()).toPose2d();
        if (alignCloseToSource) ySetpoint = -0.5;
      } else {
        targetTagPose =
            VisionConstants.aprilTagLayout.getTagPose(12).orElse(new Pose3d()).toPose2d();
        if (!alignCloseToSource) ySetpoint = -0.5;
      }
    }

    return targetTagPose.plus(new Transform2d(0.4, ySetpoint, new Rotation2d()));
  }
}
