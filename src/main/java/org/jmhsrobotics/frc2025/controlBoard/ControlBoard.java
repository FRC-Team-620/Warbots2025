package org.jmhsrobotics.frc2025.controlBoard;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public interface ControlBoard {

  // ========Driver Controls========
  public double rotation();

  public double translationX();

  public double translationY();

  public double alignLeft();

  public double alignRight();

  public Trigger resetForward();

  public Trigger turboMode();

  public Trigger autoAlignBarge();

  public Trigger autoAlignAlgaeIntake();

  public Trigger AdjustAlignBargeLeft();

  public Trigger AdjustAlignBargeRight();

  public Trigger L1AutoAlign();

  public Trigger TeleopAutoScore();

  // =======Operator Controls=======

  public DoubleSupplier intakeCoral();

  public DoubleSupplier extakeCoral();

  public Trigger intakeCoralFromIndexer();

  public Trigger placeCoralLevel1();

  public Trigger placeCoralLevel2();

  public Trigger placeCoralLevel3();

  public Trigger placeCoralLevel4();

  public Trigger altPlaceCoralLevel4();

  public Trigger scoreAlgaeProcesser();

  public Trigger algaeIntermediateSetpoint();

  public Trigger prepareAlgaeBarge();

  public Trigger scoreAlgaeBarge();

  public Trigger elevatorIntakeCoral();

  public Trigger takeAlgaeLevel2();

  public Trigger takeAlgaeLevel3();

  public Trigger takeAlgaeQTip();

  public Trigger changeModeLeft();

  public Trigger changeModeRight();

  public Trigger UnOverrideControlMode();

  public Trigger zeroElevator();

  public Trigger skipAutoScoreEast();

  public Trigger skipAutoScoreWest();

  public Trigger revertAutoScoreEast();

  public Trigger revertAutoScoreWest();
}
