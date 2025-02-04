package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;

public class ChangeMode extends Command {
  private Intake intake;

  public ChangeMode(Intake intake) {
    this.intake = intake;
  }
}
