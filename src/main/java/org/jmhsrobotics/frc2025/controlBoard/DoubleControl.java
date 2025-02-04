package org.jmhsrobotics.frc2025.controlBoard;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.util.ControllerMonitor;

public class DoubleControl implements ControlBoard {
  public CommandXboxController driver = new CommandXboxController(0);
  public CommandXboxController operator = new CommandXboxController(1);
  private Intake intake;

  public DoubleControl(Intake intake) {
    ControllerMonitor.addController(this.operator.getHID(), "Operator");
    ControllerMonitor.addController(this.driver.getHID(), "Driver");
    this.intake = intake;
  }

  private Trigger nop =
      new Trigger(
          () -> {
            return false;
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

  public Trigger intakeCoral() {
    return operator.leftTrigger();
  }

  public Trigger extakeCoral() {
    return operator.rightTrigger();
  }

  public Trigger placeCoralLevel1() {
    if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;

    return operator.a();
  }

  public Trigger placeCoralLevel2() {
    if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;

    return operator.b();
  }

  public Trigger placeCoralLevel3() {
    if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;

    return operator.x();
  }

  public Trigger placeCoralLevel4() {
    if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;
    return operator.y();
  }

  public Trigger scoreAlgaeProcesser() {
    if (intake.getMode() != Constants.ModeConstants.kAlgae) return nop;

    return operator.a();
  }

  public Trigger scoreAlgaeBarge() {
    if (intake.getMode() != Constants.ModeConstants.kAlgae) return nop;

    return operator.y();
  }

  public Trigger elevatorIntakeCoral() {
    if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;

    return operator.a();
  }

  public Trigger takeAlgaeLevel2() {
    if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;

    return operator.b();
  }

  public Trigger takeAlgaeLevel3() {
    if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;

    return operator.x();
  }

  public Trigger takeAlgaeQTip() {
    if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;

    return operator.y();
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
}
