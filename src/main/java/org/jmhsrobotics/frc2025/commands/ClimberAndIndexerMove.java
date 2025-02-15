package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.climber.Climber;

public class ClimberAndIndexerMove extends Command {
  private Climber climber;
  private double climberSpeedDutyCycle;
  private double indexerAngleDegrees;

  public ClimberAndIndexerMove(
      Climber climber, double climberSpeedDutyCycle, double indexerAngleDegrees) {
    this.climber = climber;
    this.climberSpeedDutyCycle = climberSpeedDutyCycle;
    this.indexerAngleDegrees = indexerAngleDegrees;

    addRequirements(climber);
  }

  @Override
  public void execute() {
    this.climber.set(climberSpeedDutyCycle, indexerAngleDegrees);
  }

  @Override
  public void initialize() {
    this.climber.set(0, indexerAngleDegrees);
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.climber.set(0, indexerAngleDegrees);
  }
}
