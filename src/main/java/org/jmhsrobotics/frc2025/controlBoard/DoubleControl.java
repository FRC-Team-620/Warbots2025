package org.jmhsrobotics.frc2025.controlBoard;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.util.ControllerMonitor;
import org.littletonrobotics.junction.Logger;

public class DoubleControl implements ControlBoard {
  public CommandXboxController driver = new CommandXboxController(0);
  public CommandXboxController operator = new CommandXboxController(1);
  private Intake intake;

  public DoubleControl(Intake intake) {
    ControllerMonitor.addController(this.operator.getHID(), "Operator");
    ControllerMonitor.addController(this.driver.getHID(), "Driver");
    this.intake = intake;
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

  // ========Driver Controls========

  public double rotation() {
    return driver.getRightX();
  }

  public double translationX() {
    return driver.getLeftX();
  }

  public double translationY() {
    return driver.getLeftY();
  }

  public Trigger resetForward() {
    return driver.rightBumper();
  }

  public Trigger turboMode() {
    return driver.leftBumper();
  }

  // =======Operator Controls=======

  public DoubleSupplier intakeCoral() {
    // return driver.leftTrigger();
    return () -> operator.getLeftTriggerAxis();
  }

  public DoubleSupplier extakeCoral() {
    return () -> operator.getRightTriggerAxis();
  }

  public Trigger placeCoralLevel1() {
    //  if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;

    return operator.a().and(coralMode);
  }

  public Trigger placeCoralLevel2() {
    //  if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;

    return operator.b().and(coralMode);
  }

  public Trigger placeCoralLevel3() {
    //  if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;

    return operator.x().and(coralMode);
  }

  public Trigger placeCoralLevel4() {
    //  if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;
    return operator.y().and(coralMode);
  }

  public Trigger scoreAlgaeProcesser() {
    //  if (intake.getMode() != Constants.ModeConstants.kAlgae) return nop;

    return (operator.a().or(operator.b())).and(algaeMode);
  }

  public Trigger scoreAlgaeBarge() {
    //  if (intake.getMode() != Constants.ModeConstants.kAlgae) return nop;

    return (operator.y().or(operator.x())).and(algaeMode);
  }

  public Trigger elevatorIntakeCoral() {
    //  if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;

    return operator.a().and(searchMode);
  }

  public Trigger takeAlgaeLevel2() {
    //  if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;

    return operator.b().and(searchMode);
  }

  public Trigger takeAlgaeLevel3() {
    //  if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;

    return operator.x().and(searchMode);
  }

  public Trigger takeAlgaeQTip() {
    //  if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;

    return operator.y().and(searchMode);
  }

  public Trigger climbUp() {
    return operator.povUp();
  }

  public Trigger climbDown() {
    return operator.povDown();
  }

  public Trigger indexerUp() {
    return operator.leftStick();
  }

  public Trigger indexerDown() {
    return operator.rightStick();
  }

  public Trigger changeModeLeft() {
    return operator.leftBumper();
  }

  public Trigger changeModeRight() {
    return operator.rightBumper();
  }

  public Trigger resetIndexer() {
    return operator.povLeft();
  }
}
