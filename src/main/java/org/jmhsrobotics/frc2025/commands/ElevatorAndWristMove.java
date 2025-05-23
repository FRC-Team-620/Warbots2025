package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class ElevatorAndWristMove extends SequentialCommandGroup {

  /**
   * Moves the elevator and wrist to the desired setpoints as fast as possible while preventing
   * conflicts
   *
   * <p>the standard condition is that the wrist and elevator move in parallel, but other commands
   * are added before hand for many special cases which ensure the safety of the subsystems
   *
   * @param elevator
   * @param wrist
   * @param elevatorGoalMeters
   * @param wristGoalDegrees
   */
  public ElevatorAndWristMove(
      Elevator elevator, Wrist wrist, double elevatorGoalMeters, double wristGoalDegrees) {
    addCommands(
        // all within a sequential command group so that i can easily prevent anything from
        // happening if the setpoints dont change at all
        new SequentialCommandGroup(
                // if wrist angle is lower than the safe angle, first move the wrist to the safe
                // angle
                new WristMoveTo(wrist, Constants.WristConstants.kSafeAngleDegrees)
                    .onlyIf(
                        () ->
                            wrist.getPositionDegrees()
                                <= Constants.WristConstants.kSafeAngleDegrees),

                // if wrist setpoint is at an angle less than the safe angle, move the elevator and
                // wrist to
                // safe points, then the wrist to the final angle at the end
                new ParallelCommandGroup(
                        new ElevatorMoveTo(elevator, elevatorGoalMeters),
                        new WristMoveTo(wrist, Constants.WristConstants.kSafeAngleDegrees))
                    .onlyIf(() -> wristGoalDegrees < Constants.WristConstants.kSafeAngleDegrees),

                // if at algae intake l3 setpoint and moving below algae l2 setpoint, move elevator
                // to algae
                // l2 setpoint and then move in parallel
                new ElevatorMoveTo(elevator, Constants.ElevatorConstants.kAlgaeIntakeL2Meters)
                    .onlyIf(
                        () ->
                            elevator.getSetpoint()
                                    == Constants.ElevatorConstants.kAlgaeIntakeL3Meters
                                && elevatorGoalMeters
                                    < Constants.ElevatorConstants.kAlgaeIntakeL2Meters
                                && wristGoalDegrees
                                    != Constants.WristConstants.kRotationAlgaeDegrees),

                // if the elevator setpoint is L4, move the wrist to the safe position, then
                // elevator up
                new SequentialCommandGroup(
                        new WristMoveTo(wrist, Constants.WristConstants.kSafeAngleDegrees),
                        new ElevatorMoveTo(elevator, elevatorGoalMeters))
                    .onlyIf(
                        () ->
                            elevatorGoalMeters == Constants.ElevatorConstants.kLevel4Meters
                                && elevator.getSetpoint()
                                    != Constants.ElevatorConstants.kAltLevel4Meters),

                // if going from L4 to intaking algea, move the wrist to safe angle, then move wrist
                // and
                // elevator to goal in parallel
                new WristMoveTo(wrist, Constants.WristConstants.kSafeAngleDegrees)
                    .onlyIf(
                        () ->
                            (elevatorGoalMeters == Constants.ElevatorConstants.kAlgaeIntakeL2Meters
                                    || elevatorGoalMeters
                                        == Constants.ElevatorConstants.kAlgaeIntakeL3Meters)
                                && elevator.getSetpoint()
                                    == Constants.ElevatorConstants.kLevel4Meters),

                // base case: runs both parallel for fastest speed if extraneous conditions are not
                // met
                new ParallelCommandGroup(
                    new WristMoveTo(wrist, wristGoalDegrees),
                    new ElevatorMoveTo(elevator, elevatorGoalMeters)))
            .onlyIf(
                () ->
                    elevator.getSetpoint() != elevatorGoalMeters
                        || wrist.getSetpoint() != wristGoalDegrees));
  }
}
