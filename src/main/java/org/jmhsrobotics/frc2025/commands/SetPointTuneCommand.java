package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

/**
 * This command allows for live tuning of the elevator and wrist setpoints by editing the
 * NetworkTables (NT) values. It reads two keys: "tune/wristSetpoint_deg_w" for the wrist and
 * "tune/elevatorSetpoint_M_w" for the elevator.
 */
public class SetPointTuneCommand extends Command {
  private final Elevator elevator;
  private final Wrist wrist;
  private static final String WRIST_NT_KEY = "tune/wristSetpoint_deg_w";
  private static final String ELEVATOR_NT_KEY = "tune/elevatorSetpoint_M_w";

  public SetPointTuneCommand(Elevator elevator, Wrist wrist) {
    this.elevator = elevator;
    this.wrist = wrist;

    addRequirements(this.elevator, this.wrist);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber(WRIST_NT_KEY, wrist.getPositionDegrees());
    SmartDashboard.putNumber(ELEVATOR_NT_KEY, elevator.getHeight());
    wrist.setSetpoint(wrist.getPositionDegrees());
    elevator.setSetpoint(elevator.getHeight());
  }

  @Override
  public void execute() {
    double wristSetPoint = SmartDashboard.getNumber(WRIST_NT_KEY, wrist.getPositionDegrees());
    double elevatorSetPoint = SmartDashboard.getNumber(ELEVATOR_NT_KEY, elevator.getHeight());
    wrist.setSetpoint(wristSetPoint);
    elevator.setSetpoint(elevatorSetPoint);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    wrist.setSetpoint(wrist.getPositionDegrees());
    elevator.setSetpoint(elevator.getHeight());
  }
}
