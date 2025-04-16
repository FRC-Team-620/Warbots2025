package org.jmhsrobotics.frc2025.commands.autoAlign;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.led.LED;
import org.jmhsrobotics.frc2025.subsystems.vision.Vision;
import org.jmhsrobotics.frc2025.util.ProfiledPIDControllerCustom;
import org.jmhsrobotics.frc2025.util.TrapezoidProfileCustom.Constraints;
import org.littletonrobotics.junction.Logger;

public class AlignReefSetAngleProf extends Command {
  private final Drive drive;
  private final Vision vision;
  private final LED led;
  private final Elevator elevator;
  private Timer timer = new Timer();
  // private final PIDController xController = new PIDController(0.525, 0, 0.01);
  // private final PIDController yController = new PIDController(0.525, 0, 0.01);
  public static final ProfiledPIDControllerCustom distController =
      new ProfiledPIDControllerCustom(4, 0, 0, new Constraints(6, 6, 3));
  private final PIDController thetaController = new PIDController(0.01, 0, 0);

  private double thetaGoalDegrees = 0; // Janky only work for one angle now

  private LEDPattern progressPattern;
  private double initialDistance = 3;
  private double currentDistance = 3;
  private boolean hasReset = false;

  private int targetTagId;
  private Pose3d lastTagPose = null;
  private Transform2d goalTransform;
  private Pose3d tagPose;

  // boolean for if bot should align left or right
  private boolean isLeft = true;

  public AlignReefSetAngleProf(
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

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    this.hasReset = false;
    thetaController.reset();
    thetaController.enableContinuousInput(-180, 180);

    this.goalTransform = AutoAlign.calculateReefTransform(this.elevator.getSetpoint(), isLeft);
    this.targetTagId = AutoAlign.adjustTagID(this.targetTagId);
    this.thetaGoalDegrees = AutoAlign.calculateGoalAngleFromId(this.targetTagId);

    timer.reset();
    timer.start();
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
    // new Transform3d(this.goalTransform.getX(), this.goalTransform.getY(), 0,
    // this.goalTransform.getRotation()
    tagPose =
        tagPose.plus(
            new Transform3d(
                this.goalTransform.getX(),
                this.goalTransform.getY(),
                -tagPose.getZ(),
                new Rotation3d(this.goalTransform.getRotation())));
    if (tagPose != null) {

      lastTagPose =
          new Pose3d(drive.getPose())
              .plus(new Transform3d(tagPose.getTranslation(), tagPose.getRotation()));

      this.currentDistance =
          new Pose3d(drive.getPose()).getTranslation().getDistance(lastTagPose.getTranslation());
      if (!hasReset) {
        AlignReefSetAngleProf.distController.reset(currentDistance);
        this.hasReset = true;
      }

      // calculate chassis speeds based on tag pose relative to bot and goal transformation from
      // that tag pose
      ChassisSpeeds outputSpeeds = new ChassisSpeeds();

      double speedScalar = distController.calculate(currentDistance, 0);
      Translation2d direction =
          drive
              .getPose()
              .getTranslation()
              .minus(new Translation2d(lastTagPose.getX(), lastTagPose.getY()));
      // TODO: not sure why we need to do this
      // direction = (DriverStation.)
      direction =
          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
              ? direction
              : direction.rotateBy(new Rotation2d(Units.degreesToRadians(180)));
      direction = direction.div(direction.getNorm()); // Convert to Unit Vector
      direction = direction.times(speedScalar);
      outputSpeeds = new ChassisSpeeds(direction.getX(), direction.getY(), 0);
      // outputSpeeds =
      // AutoAlign.getReefAlignSpeeds(
      //     tagPose, this.goalTransform, this.xController, this.yController);
      // calculate theta speeds
      outputSpeeds =
          outputSpeeds.plus(
              AutoAlign.getAutoAlignThetaSpeeds(
                  thetaController, this.thetaGoalDegrees, drive.getRotation()));

      Logger.recordOutput("Align Reef/Target Tag Pose1", lastTagPose);
      Logger.recordOutput("Align Reef/Target Tag ID1", this.targetTagId);
      Logger.recordOutput("Align Reef/Target Angle1", this.thetaGoalDegrees);
      Logger.recordOutput("Align Reef/Distance From Reef", this.currentDistance);
      Logger.recordOutput("Align Reef/Profiled Loop Speed Scalar", Math.abs(speedScalar));
      Logger.recordOutput(
          "Align Reef/Profiled Loop Setpoint Position",
          Math.abs(AlignReefSetAngleProf.distController.getSetpoint().position));
      Logger.recordOutput(
          "Align Reef/Profiled Loop Setpoint Velocity",
          Math.abs(AlignReefSetAngleProf.distController.getSetpoint().velocity));

      outputSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              outputSpeeds,
              DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                  ? drive.getRotation().plus(new Rotation2d(Math.PI))
                  : drive.getRotation());
      // calculates the distance from target for the LED progress pattern
      drive.runVelocity(outputSpeeds);

      // if (currentDistance > Units.inchesToMeters(1))
      //   drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(outputSpeeds,
      // drive.getRotation()));
      // led.setPattern(progressPattern);
    }
  }

  @Override
  public boolean isFinished() {
    return this.currentDistance < Units.inchesToMeters(1.5);
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Align/Profiled Align Time", timer.get());
    drive.stop();
  }
}
