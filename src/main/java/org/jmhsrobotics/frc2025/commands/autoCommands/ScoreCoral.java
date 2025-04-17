package org.jmhsrobotics.frc2025.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;

public class ScoreCoral extends Command {
  private Intake intake;
  private Elevator elevator;
  private Timer timer = new Timer();

  private double commandEndTime = 0.25;

  public ScoreCoral(Intake intake, Elevator elevator) {
    this.intake = intake;
    this.elevator = elevator;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    if (elevator.getSetpoint() == Constants.ElevatorConstants.kLevel1Meters) commandEndTime = 0.25;
    else if (elevator.getSetpoint() == Constants.ElevatorConstants.kLevel2Meters
        || elevator.getSetpoint() == Constants.ElevatorConstants.kLevel3Meters)
      commandEndTime = 0.2;
    else commandEndTime = 0.15;
  }

  @Override
  public void execute() {
    if (elevator.getSetpoint() == Constants.ElevatorConstants.kLevel1Meters) intake.set(-0.2);
    else if (elevator.getSetpoint() == Constants.ElevatorConstants.kLevel2Meters
        || elevator.getSetpoint() == Constants.ElevatorConstants.kLevel3Meters) intake.set(0.4);
    else intake.set(Constants.IntakeConstants.kCoralIntakeSpeedDutyCycle * 1.5);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(commandEndTime);
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0);
  }
}
