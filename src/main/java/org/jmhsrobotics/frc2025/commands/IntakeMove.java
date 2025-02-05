package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;

public class IntakeMove extends Command {
  private Intake intake;
  private double speedDutyCycle;

  public IntakeMove(Intake intake, double speedDutyCycle) {
    this.intake = intake;
    this.speedDutyCycle = speedDutyCycle;

    addRequirements(this.intake);
  }

  @Override
  public void execute() {
    this.intake.set(speedDutyCycle);
  }

  @Override
  public void initialize() {
    this.intake.set(0);
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.intake.set(0);
  }
}
