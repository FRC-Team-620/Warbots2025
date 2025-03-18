package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class AlignSource extends Command {
  private Drive drive;
  private Pose2d goalPose;
  private boolean alignCloseToSource;

  private final PIDController xController = new PIDController(0.3, 0, 0);
  private final PIDController yController = new PIDController(0.3, 0, 0);
  private final PIDController thetaController = new PIDController(0.1, 0, 0);

  public AlignSource(Drive drive, boolean alignCloseToSource) {
    this.drive = drive;
    this.alignCloseToSource = alignCloseToSource;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    this.goalPose = calculateReefSetpoints(drive, alignCloseToSource);
    xController.reset();
    yController.reset();
    thetaController.reset();

    xController.setSetpoint(this.goalPose.getX());
    yController.setSetpoint(this.goalPose.getY());
    thetaController.setSetpoint(this.goalPose.getRotation().getDegrees());
  }

  @Override
  public void execute() {
    this.goalPose = calculateReefSetpoints(drive, alignCloseToSource);
    drive.runVelocity(
        calculateSourceAutoAlignSpeeds(
            this.drive, this.goalPose, this.xController, this.yController, this.thetaController));
    Logger.recordOutput("Align Source/Goal Pose", this.goalPose);
  }

  @Override
  public boolean isFinished() {
    Transform2d distance = drive.getPose().minus(this.goalPose);
    // return distance.getX() < Units.inchesToMeters(0.5)
    //     && distance.getY() < Units.inchesToMeters(1)
    //     && distance.getRotation().getDegrees() < 3;
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  /**
   * calculates the coordinate setpoints for source align based drivetrain position and if the auto
   * align should be on the end further or closer to the drivers
   *
   * @param drive
   * @param alignCloseToSource
   * @return double array with index 0 being the x coordinate setpoint and index 1 being the y
   *     coordinate setpoint
   */
  public static Pose2d calculateReefSetpoints(Drive drive, boolean alignCloseToSource) {
    if (drive.getPose().getY() < 4) {
      if (alignCloseToSource) {
        return new Pose2d(0.62, 1.25, new Rotation2d(Units.degreesToRadians(54)));
      } else {
        return new Pose2d(1.55, 0.56, new Rotation2d(Units.degreesToRadians(54)));
      }
    } else {
      if (alignCloseToSource) {
        return new Pose2d(0.62, 6.75, new Rotation2d(Units.degreesToRadians(-54)));
      } else {
        return new Pose2d(1.55, 7.45, new Rotation2d(Units.degreesToRadians(-54)));
      }
    }
  }

  /**
   * calculates chassis speeds output for source auto align based on a setpoint and PID controllers
   * passed in
   *
   * @param drive
   * @param setpoint
   * @param xController
   * @param yController
   * @param thetaController
   * @return ChassisSpeeds object with x, y, and angular speeds
   */
  public static ChassisSpeeds calculateSourceAutoAlignSpeeds(
      Drive drive,
      Pose2d setpoint,
      PIDController xController,
      PIDController yController,
      PIDController thetaController) {

    double xOutput = xController.calculate(drive.getPose().getX(), setpoint.getX());
    double yOutput = yController.calculate(drive.getPose().getY(), setpoint.getY());
    double thetaOutput =
        thetaController.calculate(
            drive.getPose().getRotation().getDegrees(), setpoint.getRotation().getDegrees());

    ChassisSpeeds outputSpeeds =
        new ChassisSpeeds(
            xOutput * drive.getMaxLinearSpeedMetersPerSec(),
            yOutput * drive.getMaxLinearSpeedMetersPerSec(),
            thetaOutput * drive.getMaxAngularSpeedRadPerSec());

    return outputSpeeds;
  }
}
