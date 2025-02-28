package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.vision.Vision;

public class AlignReef extends Command {
  private final Drive drive;
  private final Vision vision;

  private final PIDController xController = new PIDController(1, 0, 0);
  private final PIDController yController = new PIDController(1, 0, 0);
  private final PIDController thetaController = new PIDController(0.05, 0, 0);
  private double xGoalMeters = 0.5;
  private double yGoalMeters = 0;
  private final double thetaGoalDegrees = 60; // Janky only work for one angle now

  public AlignReef(Drive drive, Vision vision) {
    addRequirements(drive);
    this.drive = drive;
    this.vision = vision;
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    thetaController.reset();

    xController.setSetpoint(xGoalMeters);
    yController.setSetpoint(yGoalMeters);
    thetaController.setSetpoint(thetaGoalDegrees);
    thetaController.enableContinuousInput(-180, 180);
    drive.stop();
  }

  @Override
  public void execute() {
    Pose3d tag = null; // TODO: handle seeing more than one reef tag
    for (var target : vision.getTagPoses(0)) { // TODO: Handle more than one camera
      // if(target.id() )
      if (target.id() == 17) { // TODO: janky only work for one tag for now
        tag = target.pose();
      }
      //   if (Arrays.stream(Constants.kReefAprilTags).anyMatch(elem -> elem == 12)) {
      //     tag = target.pose();
      //   }
    }
    System.out.println(tag);
    if (tag != null) {
      double theta = -Math.toDegrees(Math.atan2(tag.getY(), tag.getX()));
      double xdist = tag.getX();
      double ydist = tag.getY();
      var x = -xController.calculate(xdist);
      var y = -yController.calculate(ydist);
      var thetaOut =
          thetaController.calculate(drive.getPose().getRotation().getDegrees())
              * 0.1; // Janky clamping todo remove
      var speed =
          new ChassisSpeeds(
              x * drive.getMaxLinearSpeedMetersPerSec(),
              y * drive.getMaxLinearSpeedMetersPerSec(),
              thetaOut * drive.getMaxAngularSpeedRadPerSec());
      drive.runVelocity(speed);
    } else {
      drive.stop();
    }
  }
}
