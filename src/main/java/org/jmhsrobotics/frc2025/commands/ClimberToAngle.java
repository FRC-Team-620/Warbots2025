package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.climber.Climber;
import org.jmhsrobotics.frc2025.util.CheckTolerance;

public class ClimberToAngle extends Command {
  private Climber climber;
  private Double goalAngle;
  private double speedDutyCycle;

  public ClimberToAngle(Climber climber, double angleDegrees) {
    this.climber = climber;
    this.goalAngle = angleDegrees;

    addRequirements(climber);
  }

  @Override
  public void initialize() {
    if (goalAngle < climber.getAngleDegrees()) this.speedDutyCycle = 0.5;
    else this.speedDutyCycle = -0.5;
  }

  @Override
  public void execute() {
    this.climber.setSpeedDutyCycle(speedDutyCycle);
  }

  @Override
  public boolean isFinished() {
    return CheckTolerance.atGoalTolerance(goalAngle, climber.getAngleDegrees(), 1);
  }

  @Override
  public void end(boolean interrupted) {
    climber.setSpeedDutyCycle(0);
  }
}
