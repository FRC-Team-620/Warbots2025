package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class ScoreCoralLevel4 extends SequentialCommandGroup {

  public ScoreCoralLevel4(Elevator elevator, Wrist wrist, Intake intake) {
    // TODO: Tune elevator setpoint values and put them into constants
    addCommands(
        new ParallelCommandGroup(
            new ElevatorMoveTo(elevator, 1.4), Commands.run(() -> intake.set(0.2), intake)), );
  }
}
