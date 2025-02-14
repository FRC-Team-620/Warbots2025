package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;

public class IntakeMove extends Command {
  private Intake intake;
  private DoubleSupplier leftTriggerAxis;
  private DoubleSupplier rightTriggerAxis;

  public IntakeMove(
      Intake intake, DoubleSupplier leftTriggerAxis, DoubleSupplier rightTriggerAxis) {
    this.intake = intake;
    this.leftTriggerAxis = leftTriggerAxis;
    this.rightTriggerAxis = rightTriggerAxis;

    addRequirements(this.intake);
  }

  @Override
  public void execute() {
    double rightTrigger = rightTriggerAxis.getAsDouble();
    double leftTrigger = leftTriggerAxis.getAsDouble();
    if (rightTrigger >= leftTrigger) this.intake.set(-(Math.pow(rightTrigger, 4)));
    else this.intake.set(Math.pow(leftTrigger, 4));
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
