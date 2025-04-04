package org.jmhsrobotics.frc2025.commands.autoAlign;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class AlignBarge extends Command {
  private Drive drive;
  private Pose2d goalPose;
  private double ySetpointAdd = 0;

  private final PIDController xController = new PIDController(0.45, 0, 0.005);
  private final PIDController yController = new PIDController(0.45, 0, 0.005);
  private final PIDController thetaController = new PIDController(0.01, 0, 0);

  private Trigger leftPOVButton;
  private Trigger rightPOVButton;

  public AlignBarge(Drive drive, Trigger left, Trigger right) {
    this.drive = drive;
    this.leftPOVButton = left;
    this.rightPOVButton = right;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    this.goalPose = calculateSetpoints();
    xController.reset();
    yController.reset();
    thetaController.reset();
    thetaController.enableContinuousInput(-180, 180);

    xController.setSetpoint(this.goalPose.getX());
    yController.setSetpoint(this.goalPose.getY());
    thetaController.setSetpoint(this.goalPose.getRotation().getDegrees());
  }

  @Override
  public void execute() {

    if (leftPOVButton.getAsBoolean())
      ySetpointAdd =
          (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) ? -0.03 : 0.03;
    else if (rightPOVButton.getAsBoolean())
      ySetpointAdd =
          (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) ? 0.03 : -0.03;
    else ySetpointAdd = 0;

    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
      goalPose =
          new Pose2d(
              goalPose.getX(),
              MathUtil.clamp(goalPose.getY() + ySetpointAdd, 0.4, 3.48),
              goalPose.getRotation());
    else
      goalPose =
          new Pose2d(
              goalPose.getX(),
              MathUtil.clamp(goalPose.getY() + ySetpointAdd, 4.56, 7.66),
              goalPose.getRotation());

    Logger.recordOutput("Barge Align/GoalPose", this.goalPose);

    drive.runVelocity(
        AutoAlign.getDriveToPoseSpeeds(
            this.drive, this.goalPose, this.xController, this.yController, this.thetaController));
    // calculateSourceAutoAlignSpeeds(
    //     this.drive, this.goalPose, this.xController, this.yController, this.thetaController));
    // Logger.recordOutput("Align Source/Goal Pose", this.goalPose);
    xController.setSetpoint(this.goalPose.getX());
    yController.setSetpoint(this.goalPose.getY());
  }

  @Override
  public boolean isFinished() {
    // Transform2d distance = drive.getPose().minus(this.goalPose);
    // return distance.getX() < Units.inchesToMeters(1)
    //  && distance.getY() < Units.inchesToMeters(1)
    // && distance.getRotation().getDegrees() < 3;
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
   * @return Pose2d
   */
  public static Pose2d calculateSetpoints() {
    double xSetpoint;
    double ySetpoint;
    double goalTheta;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      xSetpoint = 9.771;
      ySetpoint = 1.9;
      goalTheta = 0;
    } else {
      xSetpoint = 7.789;
      ySetpoint = 6.2;
      goalTheta = 180;
    }

    return new Pose2d(xSetpoint, ySetpoint, new Rotation2d(Units.degreesToRadians(goalTheta)));
  }
}
