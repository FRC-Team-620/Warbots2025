package org.jmhsrobotics.frc2025.commands.autoCommands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.commands.ElevatorAndWristMove;
import org.jmhsrobotics.frc2025.commands.FixCoralPlacement;
import org.jmhsrobotics.frc2025.commands.IntakeFromIndexer;
import org.jmhsrobotics.frc2025.commands.LEDFlashPattern;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.led.LED;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class IntakeCoralAuto extends SequentialCommandGroup {
  public IntakeCoralAuto(Elevator elevator, Wrist wrist, Intake intake, LED led) {
    addCommands(
        new ElevatorAndWristMove(
            elevator,
            wrist,
            intake,
            Constants.ElevatorConstants.kCoralIntakeMeters,
            Constants.WristConstants.kSafeAngleDegrees),
        new ParallelRaceGroup(
            new IntakeFromIndexer(wrist, intake).withTimeout(4),
            new LEDFlashPattern(
                led, LEDPattern.solid(Color.kYellow), LEDPattern.solid(Color.kWhite))),
        new FixCoralPlacement(intake, wrist).withTimeout(4));
  }
}
