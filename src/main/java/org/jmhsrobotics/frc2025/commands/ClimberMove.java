package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.climber.Climber;

public class ClimberMove extends Command {
  private Climber climber;
  private double climberSpeedDutyCycle;

  public ClimberMove(Climber climber, double climberSpeedDutyCycle) {
    this.climber = climber;
    this.climberSpeedDutyCycle = climberSpeedDutyCycle;

    addRequirements(climber);
  }

  @Override
  public void execute() {
    this.climber.set(climberSpeedDutyCycle);
  }

  @Override
  public void initialize() {
    this.climber.set(0);
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.climber.set(0);
  }
}
