package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class IntakeFromIndexer extends Command {
  private Wrist wrist;
  private Intake intake;

  private Timer timer = new Timer();

  public IntakeFromIndexer(Wrist wrist, Intake intake) {
    this.wrist = wrist;
    this.intake = intake;

    addRequirements(wrist, intake);
  }

  @Override
  public void initialize() {
    intake.set(0);
    wrist.setSetpoint(Constants.WristConstants.kRotationIntakeCoralDegrees);
    timer.reset();
    System.out.println("Starting Intake from indexer");
  }

  @Override
  public void execute() {
    intake.set(Constants.IntakeConstants.kCoralIntakeIndexerSpeedDutyCycle);
  }

  @Override
  public boolean isFinished() {
    return intake.isCoralInIntake();
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0);
    System.out.println("Intake From Indexer Complete: " + interrupted);
  }
}
