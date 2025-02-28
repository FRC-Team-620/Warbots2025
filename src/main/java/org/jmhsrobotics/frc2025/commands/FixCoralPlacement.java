package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class FixCoralPlacement extends Command {
  private Intake intake;
  private Wrist wrist;

  private boolean hasPassedSensor = false;

  public FixCoralPlacement(Intake intake, Wrist wrist) {
    this.intake = intake;
    this.wrist = wrist;

    addRequirements(intake, wrist);
  }

  @Override
  public void initialize() {
    intake.set(Constants.IntakeConstants.kCoralDefaultCommandSpeed);
  }

  @Override
  public void execute() {
    // If the coral is no longer in the in front of the sensor, it has passed and intake direction
    // will switch
    if (!hasPassedSensor) {
      if (!intake.isCoralInIntake()) hasPassedSensor = true;
    }

    if (hasPassedSensor) {
      intake.set(-Constants.IntakeConstants.kCoralDefaultCommandSpeed);
      wrist.setSetpoint(Constants.WristConstants.kSafeAngleDegrees);
    } else intake.set(Constants.IntakeConstants.kCoralDefaultCommandSpeed);
  }

  @Override
  public boolean isFinished() {
    // once coral comes back in from of sensor after passing
    return hasPassedSensor && intake.isCoralInIntake();
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0);
  }
}
