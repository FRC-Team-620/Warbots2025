package org.jmhsrobotics.frc2025.controlBoard;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.util.ControllerMonitor;
import org.littletonrobotics.junction.Logger;

public class DoubleControl implements ControlBoard {
  public CommandXboxController driver = new CommandXboxController(0);
  public CommandXboxController operator = new CommandXboxController(1);
  private Intake intake;
  private Elevator elevator;

  public DoubleControl(Intake intake, Elevator elevator) {
    ControllerMonitor.addController(this.operator.getHID(), "Operator");
    ControllerMonitor.addController(this.driver.getHID(), "Driver");
    this.intake = intake;
    this.elevator = elevator;
    coralMode.whileTrue(
        new InstantCommand(
                () -> {
                  Logger.recordOutput("ctlMode", "coral");
                },
                intake)
            .ignoringDisable(true));
    algaeMode.whileTrue(
        new InstantCommand(
                () -> {
                  Logger.recordOutput("ctlMode", "algae");
                },
                intake)
            .ignoringDisable(true));
    searchMode.whileTrue(
        new InstantCommand(
                () -> {
                  Logger.recordOutput("ctlMode", "search");
                },
                intake)
            .ignoringDisable(true));
  }

  private Trigger coralMode =
      new Trigger(
          () -> {
            return intake.getMode() == Constants.ModeConstants.kCoral;
          });

  private Trigger algaeMode =
      new Trigger(
          () -> {
            return intake.getMode() == Constants.ModeConstants.kAlgae;
          });

  private Trigger searchMode =
      new Trigger(
          () -> {
            return intake.getMode() == Constants.ModeConstants.kSearch;
          });

  private Trigger elevatorAtBottom =
      new Trigger(
          () -> {
            return elevator.getSetpoint() == Constants.ElevatorConstants.kCoralIntakeMeters;
          });

  // ========Driver Controls========

  @Override
  public double rotation() {
    return driver.getRightX();
  }

  @Override
  public double translationX() {
    return driver.getLeftX();
  }

  @Override
  public double translationY() {
    return driver.getLeftY();
  }

  @Override
  public Trigger resetForward() {
    return driver.rightBumper();
  }

  @Override
  public Trigger turboMode() {
    return driver.leftBumper();
  }

  // =======Operator Controls=======

  @Override
  public Trigger intakeCoralFromIndexer() {
    return operator.rightStick().and(elevatorAtBottom);
  }

  @Override
  public DoubleSupplier intakeCoral() {
    return () -> operator.getLeftTriggerAxis();
  }

  @Override
  public DoubleSupplier extakeCoral() {
    return () -> operator.getRightTriggerAxis();
  }

  @Override
  public Trigger placeCoralLevel1() {
    //  if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;
    return operator.a().and(coralMode);
  }

  @Override
  public Trigger placeCoralLevel2() {
    //  if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;
    return operator.b().and(coralMode);
  }

  @Override
  public Trigger placeCoralLevel3() {
    //  if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;
    return operator.x().and(coralMode);
  }

  @Override
  public Trigger placeCoralLevel4() {
    //  if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;
    return operator.y().and(coralMode);
  }

  @Override
  public Trigger scoreAlgaeProcesser() {
    //  if (intake.getMode() != Constants.ModeConstants.kAlgae) return nop;
    return (operator.a().or(operator.b())).and(algaeMode);
  }

  @Override
  public Trigger scoreAlgaeBarge() {
    //  if (intake.getMode() != Constants.ModeConstants.kAlgae) return nop;
    return (operator.y().or(operator.x())).and(algaeMode);
  }

  @Override
  public Trigger elevatorIntakeCoral() {
    //  if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;
    return operator.a().and(searchMode);
  }

  @Override
  public Trigger takeAlgaeLevel2() {
    //  if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;
    return operator.b().and(searchMode);
  }

  @Override
  public Trigger takeAlgaeLevel3() {
    //  if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;
    return operator.x().and(searchMode);
  }

  @Override
  public Trigger takeAlgaeQTip() {
    //  if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;
    return operator.y().and(searchMode);
  }

  @Override
  public Trigger climbUp() {
    return operator.povUp();
  }

  @Override
  public Trigger climbDown() {
    return operator.povDown();
  }

  @Override
  public Trigger indexerUp() {
    return operator.leftStick();
  }

  @Override
  public Trigger indexerDown() {
    return operator.rightStick();
  }

  @Override
  public Trigger changeModeLeft() {
    return operator.leftBumper();
  }

  @Override
  public Trigger changeModeRight() {
    return operator.rightBumper();
  }

  @Override
  public Trigger UnOverrideControlMode() {
    return operator.rightBumper().and(operator.leftBumper());
  }

  @Override
  public Trigger resetIndexer() {
    return operator.povLeft();
  }
}
