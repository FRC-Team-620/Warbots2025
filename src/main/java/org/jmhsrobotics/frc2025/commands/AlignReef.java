package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.vision.Vision;

public class AlignReef extends Command {
  private final Drive drive;
  private final Vision vision;

  private final PIDController xController = new PIDController(0.1, 0, 0);
  private final PIDController yController = new PIDController(0.1, 0, 0);
  private final PIDController thetaController = new PIDController(0.1, 0, 0);

  AlignReef(Drive drive, Vision vision) {
    addRequirements(drive);
    this.drive = drive;
    ;
    this.vision = vision;
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    thetaController.reset();
  }

  @Override
  public void execute() {}
}
