package org.jmhsrobotics.frc2025.commands.autoAlign;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.util.ProfiledPIDControllerCustom;
import org.jmhsrobotics.frc2025.util.TrapezoidProfileCustom.Constraints;
import org.littletonrobotics.junction.Logger;

public class AlignSourceProf extends Command {
  private Drive drive;
  private boolean alignCloseToStation = false;

  private boolean hasReset = false;

  public static final ProfiledPIDControllerCustom distController =
      new ProfiledPIDControllerCustom(5, 0.0, 0.0, new Constraints(6, 12, 5));
  private final PIDController thetaController = new PIDController(0.01, 0, 0);

  private Pose2d goalPose = new Pose2d();

  private final Timer timer = new Timer();

  private double currentDistance = 3;

  public AlignSourceProf(Drive drive, boolean alignCloseToStation) {
    this.drive = drive;
    this.alignCloseToStation = alignCloseToStation;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    this.goalPose = AlignSource.calculateSetpoints(drive, alignCloseToStation);

    thetaController.reset();
    thetaController.enableContinuousInput(-180, 180);
    this.hasReset = false;

    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    this.currentDistance =
        Math.sqrt(
            Math.pow(goalPose.getX() - drive.getPose().getX(), 2)
                + Math.pow(goalPose.getY() - drive.getPose().getY(), 2));

    if (!hasReset) {
      this.distController.reset(currentDistance);
      this.hasReset = true;
    }
    ChassisSpeeds outputSpeeds = new ChassisSpeeds();

    double speedScalar = distController.calculate(this.currentDistance, 0);

    Translation2d direction = drive.getPose().getTranslation().minus(goalPose.getTranslation());
    // convert to unit vector
    direction = direction.div(direction.getNorm());
    direction = direction.times(speedScalar);

    outputSpeeds = new ChassisSpeeds(direction.getX(), direction.getY(), 0);

    outputSpeeds =
        outputSpeeds.plus(
            AutoAlign.getAutoAlignThetaSpeeds(
                thetaController, goalPose.getRotation().getDegrees(), drive.getRotation()));

    outputSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(outputSpeeds, drive.getRotation());

    drive.runVelocity(outputSpeeds);

    Logger.recordOutput("Align Source Profiled/Goal Pose", this.goalPose);
    Logger.recordOutput("Align Source Profiled/Current Distance", this.currentDistance);
    Logger.recordOutput(
        "Align Source Profiled/Calculated Velocity Setpoint", Math.abs(speedScalar));
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return this.currentDistance < Units.inchesToMeters(1.25);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    Logger.recordOutput("Align Source Profiled/Command TImer", timer.get());
  }
}
