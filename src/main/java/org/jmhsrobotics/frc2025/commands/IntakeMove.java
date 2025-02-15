package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class IntakeMove extends Command {
  private Intake intake;
  private Wrist wrist;
  private DoubleSupplier leftTriggerAxis;
  private DoubleSupplier rightTriggerAxis;

  // helps determine which way the intake should move
  private boolean isInAlgaeMode;

  public IntakeMove(
      Intake intake, Wrist wrist, DoubleSupplier leftTriggerAxis, DoubleSupplier rightTriggerAxis) {
    this.intake = intake;
    this.wrist = wrist;
    this.leftTriggerAxis = leftTriggerAxis;
    this.rightTriggerAxis = rightTriggerAxis;

    addRequirements(this.intake);
  }

  @Override
  public void execute() {
    if (wrist.getSetpoint() == Constants.WristConstants.kRotationAlgaeDegrees
        || wrist.getSetpoint() == Constants.WristConstants.kRotationBargeDegrees
        || wrist.getSetpoint() == Constants.WristConstants.kRotationProcesserDegrees) {
      isInAlgaeMode = true;
    }

    double rightTrigger = rightTriggerAxis.getAsDouble();
    double leftTrigger = leftTriggerAxis.getAsDouble();

    // chooses which trigger to use based on which input is greater
    // Makes it so the intake moves a different direction based on the angle of the wrist so that
    // the right trigger is always intake
    if (rightTrigger >= leftTrigger) {
      if (isInAlgaeMode)
        this.intake.set(-rightTrigger * Constants.IntakeConstants.kMaxSpeedDutyCycle);
      else this.intake.set(rightTrigger * Constants.IntakeConstants.kMaxSpeedDutyCycle);
    } else {
      if (isInAlgaeMode)
        this.intake.set(leftTrigger * Constants.IntakeConstants.kMaxSpeedDutyCycle);
      else this.intake.set(-leftTrigger * Constants.IntakeConstants.kMaxSpeedDutyCycle);
    }

    if (rightTrigger + leftTrigger == 0) {
      if (isInAlgaeMode) {
        algaeDefaultCommand();
      } else if (wrist.getSetpoint() == Constants.WristConstants.kSafeAngleDegrees) {
      } else {
        coralDefaultCommand();
      }
    }
  }

  private void algaeDefaultCommand() {
    intake.set(Constants.IntakeConstants.kAlgaeDefaultCommandSpeed);
  }

  private void coralDefaultCommand() {
    if (intake.getCoralDistance() > 30) {
      intake.set(Constants.IntakeConstants.kCoralDefaultCommandSpeed);
    }
  }

  @Override
  public void initialize() {
    this.intake.set(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.intake.set(0);
  }
}
