package org.jmhsrobotics.frc2025.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class WristMoveToNoFinish extends Command {
  private Wrist wrist;
  private double goalDegrees;

  public WristMoveToNoFinish(Wrist wrist, double goalDegrees) {
    this.wrist = wrist;
    this.goalDegrees = goalDegrees;

    addRequirements(this.wrist);
  }

  @Override
  public void initialize() {
    this.wrist.setSetpoint(goalDegrees);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
