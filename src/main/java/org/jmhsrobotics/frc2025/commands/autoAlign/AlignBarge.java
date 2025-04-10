package org.jmhsrobotics.frc2025.commands.autoAlign;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

  private boolean isTeleop = true;
  private boolean isRedAlliance = true;
  ;

  private double yOffset = 0;

  /**
   * Aligns to the barge to a default setpoint in the middle. Can move left or right based on
   * trigger inputs, and it unending
   *
   * @param drive Drive subsystem
   * @param left Trigger that moves the setpoint to the left along the barge
   * @param right Trigger that moves the setpoint to the right along the barge
   */
  public AlignBarge(Drive drive, Trigger left, Trigger right) {
    this.drive = drive;
    this.leftPOVButton = left;
    this.rightPOVButton = right;
    this.isTeleop = true;
    addRequirements(drive);
  }

  /**
   * Align Barge for autonomous phase or when yOffset is predetermined and command should not be
   * unending
   *
   * @param drive Drive subsystem
   * @param yOffset The offset in meters on the Y axis of the goal setpoint starting from the center
   *     of the barge. positive is closer to the center
   */
  public AlignBarge(Drive drive, double yOffset) {
    this.drive = drive;
    this.isTeleop = false;
    this.yOffset = yOffset;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    this.isRedAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    this.goalPose = calculateSetpoints();

    if (!isTeleop)
      this.goalPose = goalPose.transformBy(new Transform2d(0, yOffset, new Rotation2d()));

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

    // changes goal pose based on left and right POV only if in teleop. otherwise keeps the default
    // pose
    if (isTeleop) {
      ySetpointAdd = 0;
      if (leftPOVButton.getAsBoolean()) ySetpointAdd = (isRedAlliance) ? -0.03 : 0.03;
      else if (rightPOVButton.getAsBoolean()) ySetpointAdd = (isRedAlliance) ? 0.03 : -0.03;

      if (isRedAlliance)
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
    }

    Logger.recordOutput("Barge Align/GoalPose", this.goalPose);

    drive.runVelocity(
        AutoAlign.getDriveToPoseSpeeds(
            this.drive, this.goalPose, this.xController, this.yController, this.thetaController));

    xController.setSetpoint(this.goalPose.getX());
    yController.setSetpoint(this.goalPose.getY());
  }

  @Override
  public boolean isFinished() {
    if (!isTeleop) {
      Transform2d distance = drive.getPose().minus(this.goalPose);
      return Math.abs(distance.getX()) < Units.inchesToMeters(2.5)
          && Math.abs(distance.getY()) < Units.inchesToMeters(2.5)
          && Math.abs(distance.getRotation().getDegrees()) < 3;
    }
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
  public Pose2d calculateSetpoints() {
    // first number is half the bumber length, should not change
    // second number is distance to setpoint from edge of bumper
    double xDisplacement = 17.5 + 15;
    return (isRedAlliance)
        ? new Pose2d(
            8.78 + Units.inchesToMeters(xDisplacement),
            1.9,
            new Rotation2d(Units.degreesToRadians(0)))
        : new Pose2d(
            8.78 - Units.inchesToMeters(xDisplacement),
            6.2,
            new Rotation2d(Units.degreesToRadians(180)));
  }
}
