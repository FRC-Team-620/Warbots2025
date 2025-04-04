// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.jmhsrobotics.frc2025;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.canand.CanandEventLoop;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.jmhsrobotics.frc2025.commands.DriveCommands;
import org.jmhsrobotics.frc2025.commands.DriveMeToTheMoon;
import org.jmhsrobotics.frc2025.commands.DriveTimeCommand;
import org.jmhsrobotics.frc2025.commands.ElevatorAndWristMove;
import org.jmhsrobotics.frc2025.commands.ElevatorSetZero;
import org.jmhsrobotics.frc2025.commands.FixCoralPlacement;
import org.jmhsrobotics.frc2025.commands.IndexerMove;
import org.jmhsrobotics.frc2025.commands.IntakeFromIndexer;
import org.jmhsrobotics.frc2025.commands.IntakeMove;
import org.jmhsrobotics.frc2025.commands.LEDToControlMode;
import org.jmhsrobotics.frc2025.commands.ScoreBargeWrist;
import org.jmhsrobotics.frc2025.commands.SetPointTuneCommand;
import org.jmhsrobotics.frc2025.commands.WristMoveTo;
import org.jmhsrobotics.frc2025.commands.autoAlign.AlignBarge;
import org.jmhsrobotics.frc2025.commands.autoAlign.AlignReef;
import org.jmhsrobotics.frc2025.commands.autoAlign.AlignReefSetAngle;
import org.jmhsrobotics.frc2025.commands.autoAlign.AlignSource;
import org.jmhsrobotics.frc2025.commands.autoCommands.DriveBackwards;
import org.jmhsrobotics.frc2025.commands.autoCommands.IntakeUntilCoralInIndexer;
import org.jmhsrobotics.frc2025.commands.autoCommands.ScoreCoral;
import org.jmhsrobotics.frc2025.controlBoard.ControlBoard;
import org.jmhsrobotics.frc2025.controlBoard.DoubleControl;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.drive.GyroIO;
import org.jmhsrobotics.frc2025.subsystems.drive.GyroIOBoron;
import org.jmhsrobotics.frc2025.subsystems.drive.swerve.ModuleIO;
import org.jmhsrobotics.frc2025.subsystems.drive.swerve.ModuleIOSimRev;
import org.jmhsrobotics.frc2025.subsystems.drive.swerve.ModuleIOThrifty;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.elevator.ElevatorIO;
import org.jmhsrobotics.frc2025.subsystems.elevator.SimElevatorIO;
import org.jmhsrobotics.frc2025.subsystems.elevator.VortexElevatorIO;
import org.jmhsrobotics.frc2025.subsystems.indexer.Indexer;
import org.jmhsrobotics.frc2025.subsystems.indexer.IndexerIO;
import org.jmhsrobotics.frc2025.subsystems.indexer.NeoIndexerIO;
import org.jmhsrobotics.frc2025.subsystems.indexer.SimIndexerIO;
import org.jmhsrobotics.frc2025.subsystems.intake.GrappleTimeOfFLightIO;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.intake.IntakeIO;
import org.jmhsrobotics.frc2025.subsystems.intake.NeoIntakeIO;
import org.jmhsrobotics.frc2025.subsystems.intake.SimIntakeIO;
import org.jmhsrobotics.frc2025.subsystems.intake.SimTimeOfFlightIO;
import org.jmhsrobotics.frc2025.subsystems.intake.TimeOfFLightIO;
import org.jmhsrobotics.frc2025.subsystems.led.LED;
import org.jmhsrobotics.frc2025.subsystems.vision.Vision;
import org.jmhsrobotics.frc2025.subsystems.vision.VisionConstants;
import org.jmhsrobotics.frc2025.subsystems.vision.VisionIO;
import org.jmhsrobotics.frc2025.subsystems.vision.VisionIOPhotonVision;
import org.jmhsrobotics.frc2025.subsystems.vision.VisionIOPhotonVisionSim;
import org.jmhsrobotics.frc2025.subsystems.wrist.NeoWristIO;
import org.jmhsrobotics.frc2025.subsystems.wrist.SimWristIO;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;
import org.jmhsrobotics.frc2025.subsystems.wrist.WristIO;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  private final Vision vision;
  public final Elevator elevator;
  public final Wrist wrist;
  private final ControlBoard control;
  private final LED led;
  public final Intake intake;
  public final Indexer indexer;
  private boolean isBrakeMode = true;

  // Controller

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Setup Boron Server
    CanandEventLoop.getInstance();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOBoron(),
                new ModuleIOThrifty(0),
                new ModuleIOThrifty(1),
                new ModuleIOThrifty(2),
                new ModuleIOThrifty(3));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1));

        elevator = new Elevator(new VortexElevatorIO() {});
        wrist = new Wrist(new NeoWristIO());
        intake = new Intake(new NeoIntakeIO(), new GrappleTimeOfFLightIO());
        indexer = new Indexer(new NeoIndexerIO());

        System.out.println("Mode: REAL");
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOBoron(),
                new ModuleIOSimRev(),
                new ModuleIOSimRev(),
                new ModuleIOSimRev(),
                new ModuleIOSimRev());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
        elevator = new Elevator(new SimElevatorIO());
        wrist = new Wrist(new SimWristIO());
        intake = new Intake(new SimIntakeIO() {}, new SimTimeOfFlightIO() {});
        indexer = new Indexer(new SimIndexerIO());

        System.out.println("Mode: SIM");
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        wrist = new Wrist(new WristIO() {});
        intake = new Intake(new IntakeIO() {}, new TimeOfFLightIO() {});
        indexer = new Indexer(new IndexerIO() {});

        System.out.println("Mode: DEFAULT");
        break;
    }

    this.control = new DoubleControl(intake, elevator);

    led = new LED();

    // Set up auto routines
    // PathPlanner Named Commands needs to be configured before autochoose is made
    configurePathPlanner();
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption("BaseLineAuto", new DriveTimeCommand(2.2, 0.3, drive));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
    configureDriverFeedback();
    setupSmartDashbaord();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    intake.setDefaultCommand(
        new IntakeMove(intake, wrist, control.intakeCoral(), control.extakeCoral()));

    drive.setDefaultCommand(
        new DriveMeToTheMoon(
            drive,
            vision,
            elevator,
            intake,
            wrist,
            indexer,
            () -> -control.translationY(),
            () -> -control.translationX(),
            () -> -control.rotation(),
            () -> control.alignLeft(),
            () -> control.alignRight(),
            control.autoAlignAlgaeIntake()));
    // Reset gyro to 0° when right bumper is pressed

    control
        .resetForward()
        .onTrue(
            Commands.runOnce(
                () -> {
                  boolean isRed =
                      DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get() == Alliance.Red;
                  drive.setPose(
                      new Pose2d(
                          drive.getPose().getTranslation(),
                          Rotation2d.fromDegrees(isRed ? 180 : 0)));
                },
                drive));

    control
        .placeCoralLevel1()
        .onTrue(
            new ElevatorAndWristMove(
                elevator,
                wrist,
                Constants.ElevatorConstants.kLevel1Meters,
                Constants.WristConstants.kLevel1Degrees));
    control
        .placeCoralLevel2()
        .onTrue(
            new ElevatorAndWristMove(
                elevator,
                wrist,
                Constants.ElevatorConstants.kLevel2Meters,
                Constants.WristConstants.kLevel2Degrees));
    control
        .placeCoralLevel2()
        .onTrue(
            new InstantCommand(
                () -> {
                  System.out.println("");
                }));
    control
        .placeCoralLevel1()
        .onTrue(
            new InstantCommand(
                () -> {
                  System.out.println("");
                }));
    control
        .placeCoralLevel3()
        .onTrue(
            new ElevatorAndWristMove(
                elevator,
                wrist,
                Constants.ElevatorConstants.kLevel3Meters,
                Constants.WristConstants.kLevel3Degrees));
    control
        .placeCoralLevel4()
        .onTrue(
            new ElevatorAndWristMove(
                elevator,
                wrist,
                Constants.ElevatorConstants.kLevel4Meters,
                Constants.WristConstants.kLevel4Degrees));

    control
        .scoreAlgaeProcesser()
        .onTrue(
            new ElevatorAndWristMove(
                elevator,
                wrist,
                Constants.ElevatorConstants.kProcesserMeters,
                Constants.WristConstants.kRotationProcesserDegrees));

    control.scoreAlgaeBarge().onTrue(new ScoreBargeWrist(wrist, elevator, intake));

    control
        .elevatorIntakeCoral()
        .onTrue(
            new ElevatorAndWristMove(
                elevator,
                wrist,
                Constants.ElevatorConstants.kCoralIntakeMeters,
                Constants.WristConstants.kSafeAngleDegrees));

    control
        .takeAlgaeLevel2()
        .onTrue(
            new ElevatorAndWristMove(
                elevator,
                wrist,
                Constants.ElevatorConstants.kAlgaeIntakeL2Meters,
                Constants.WristConstants.kRotationAlgaeDegrees));

    control
        .takeAlgaeLevel3()
        .onTrue(
            new ElevatorAndWristMove(
                elevator,
                wrist,
                Constants.ElevatorConstants.kAlgaeIntakeL3Meters,
                Constants.WristConstants.kRotationAlgaeDegrees));

    control
        .takeAlgaeQTip()
        .onTrue(
            new ElevatorAndWristMove(
                elevator,
                wrist,
                Constants.ElevatorConstants.kAlgaeQTipMeters,
                Constants.WristConstants.kRotationAlgaeDegrees));

    control
        .prepareAlgaeBarge()
        .onTrue(
            new ElevatorAndWristMove(
                elevator,
                wrist,
                Constants.ElevatorConstants.kPreBargeMeters,
                Constants.WristConstants.kRotationProcesserDegrees));
    control
        .intakeCoralFromIndexer()
        .onTrue(
            new SequentialCommandGroup(
                new ElevatorAndWristMove(
                    elevator,
                    wrist,
                    Constants.ElevatorConstants.kCoralIntakeMeters,
                    Constants.WristConstants.kRotationIntakeCoralDegrees),
                new IntakeFromIndexer(wrist, intake, indexer, led),
                new FixCoralPlacement(intake)));

    control
        .changeModeLeft()
        .onTrue(Commands.runOnce(() -> intake.setMode(-1), intake).ignoringDisable(true));

    control
        .changeModeRight()
        .onTrue(Commands.runOnce(() -> intake.setMode(1), intake).ignoringDisable(true));

    control
        .zeroElevator()
        .onTrue(
            new SequentialCommandGroup(
                new WristMoveTo(wrist, Constants.WristConstants.kSafeAngleDegrees),
                new ElevatorSetZero(elevator)));

    control.UnOverrideControlMode().onTrue(Commands.runOnce(() -> intake.unOverrideControlMode()));

    control.turboMode().onTrue(Commands.runOnce(() -> drive.setTurboMode(true), drive));
    control.turboMode().onFalse(Commands.runOnce(() -> drive.setTurboMode(false), drive));

    control
        .autoAlignBarge()
        .whileTrue(
            new AlignBarge(drive, control.AdjustAlignBargeLeft(), control.AdjustAlignBargeRight()));
  }

  private void configureDriverFeedback() {
    // Changes LED light status and controller rumble
    led.setDefaultCommand(new LEDToControlMode(this.led, this.intake));

    // If control mode is manually overridden, lights flash red and green(Christmas!)
    new Trigger(intake::isControlModeOverridden)
        .onTrue(
            Commands.run(
                    () -> led.setPattern(LEDPattern.solid(Color.kRed).blink(Seconds.of(0.1))), led)
                .withTimeout(1.5));

    // if control mode is un-overridden, lights will flash gold and white
    new Trigger(intake::isControlModeOverridden)
        .onFalse(
            Commands.run(
                    () -> led.setPattern(LEDPattern.solid(Color.kGold).blink(Seconds.of(0.1))), led)
                .withTimeout(1.5));

    new Trigger(drive::isAutoAlignComplete)
        .whileTrue(
            Commands.run(
                    () -> led.setPattern(LEDPattern.solid(Color.kCyan).blink(Seconds.of(0.1))), led)
                .withTimeout(1.5));
  }

  private void setupSmartDashbaord() {
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("toogleBrakeMode", getToggleBrakeCommand());
    new Trigger(RobotController::getUserButton)
        .onTrue(getToggleBrakeCommand()); // TODO: disable when in a match?

    SmartDashboard.putData("cmd/Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData(
        "cmd/SwitchModeLeft", Commands.runOnce(() -> intake.setMode(-1), intake));
    SmartDashboard.putData(
        "cmd/SwitchModeRight", Commands.runOnce(() -> intake.setMode(1), intake));
    SmartDashboard.putData("cmd/RunElevatorZeroCommand", new ElevatorSetZero(elevator));
    SmartDashboard.putData("cmd/SetPointTuneCommand", new SetPointTuneCommand(elevator, wrist));
    SmartDashboard.putData(
        "cmd/Align Reef Left", new AlignReef(drive, vision, led, elevator, true).withTimeout(5));
    SmartDashboard.putData(
        "cmd/Align Reef Right", new AlignReef(drive, vision, led, elevator, false).withTimeout(5));
    SmartDashboard.putData("Fix Coral Placement", new FixCoralPlacement(intake));

    SmartDashboard.putData("Scheduler2", CommandScheduler.getInstance());
    SmartDashboard.putData("cmd/Score Coral", new ScoreCoral(intake).withTimeout(0.15));
    SmartDashboard.putData("cmd/Align Source Close", new AlignSource(drive, true));
    SmartDashboard.putData("cmd/Align Source Far", new AlignSource(drive, false));
    SmartDashboard.putData("cmd/Score Barge Wrist", new ScoreBargeWrist(wrist, elevator, intake));

    SmartDashboard.putData(
        "cmd/Intake Indexer",
        new SequentialCommandGroup(
            new ElevatorAndWristMove(
                elevator,
                wrist,
                Constants.ElevatorConstants.kCoralIntakeMeters,
                Constants.WristConstants.kRotationIntakeCoralDegrees),
            new IntakeFromIndexer(wrist, intake, indexer, led),
            new FixCoralPlacement(intake)));
    SmartDashboard.putData(
        "cmd/Align Preset Southwest",
        new AlignReefSetAngle(drive, vision, led, elevator, true, 19));
    SmartDashboard.putData(
        "cmd/Align Preset Southwest Right",
        new AlignReefSetAngle(drive, vision, led, elevator, false, 19));
    SmartDashboard.putData(
        "cmd/Align Preset Northwest",
        new AlignReefSetAngle(drive, vision, led, elevator, false, 20));
    SmartDashboard.putData("cmd/Run Indexer", new IndexerMove(indexer));
    SmartDashboard.putData(
        "cmd/Run Intake Sequence",
        new SequentialCommandGroup(
            new ElevatorAndWristMove(
                elevator,
                wrist,
                Constants.ElevatorConstants.kCoralIntakeMeters,
                Constants.WristConstants.kRotationIntakeCoralDegrees),
            new IntakeFromIndexer(wrist, intake, indexer, led),
            new FixCoralPlacement(intake)));
    SmartDashboard.putData(
        "cmd/Align Barge",
        new AlignBarge(drive, control.AdjustAlignBargeLeft(), control.AdjustAlignBargeRight()));
  }

  private void configurePathPlanner() {
    // Elevator and Wrist Command
    NamedCommands.registerCommand(
        "Elevator And Wrist L4",
        new ElevatorAndWristMove(
            elevator,
            wrist,
            Constants.ElevatorConstants.kLevel4Meters,
            Constants.WristConstants.kLevel4Degrees));
    NamedCommands.registerCommand(
        "Elevator And Wrist L3",
        new ElevatorAndWristMove(
            elevator,
            wrist,
            Constants.ElevatorConstants.kLevel3Meters,
            Constants.WristConstants.kLevel3Degrees));
    NamedCommands.registerCommand(
        "Elevator And Wrist L2",
        new ElevatorAndWristMove(
            elevator,
            wrist,
            Constants.ElevatorConstants.kLevel2Meters,
            Constants.WristConstants.kLevel2Degrees));
    NamedCommands.registerCommand(
        "Elevator And Wrist Coral Intake",
        new ElevatorAndWristMove(
            elevator,
            wrist,
            Constants.ElevatorConstants.kCoralIntakeMeters,
            Constants.WristConstants.kSafeAngleDegrees));
    // Intake Commands
    // TODO: Intake Coral command needs to be updated once updated intake control is merged to
    // master to also run the fix coral placement command

    // timeouts needed for simulation since they will never end without simulated game piece pickup
    if (Robot.isSimulation()) {
      NamedCommands.registerCommand(
          "Intake Until Indexer Has Coral",
          new IntakeUntilCoralInIndexer(wrist, intake, indexer, led).withTimeout(1));

      NamedCommands.registerCommand(
          "Finish Coral Intake", new IntakeFromIndexer(wrist, intake, indexer, led).withTimeout(2));

      NamedCommands.registerCommand(
          "Fix Coral Placement", new FixCoralPlacement(intake).withTimeout(1.2));
    } else {
      NamedCommands.registerCommand(
          "Intake Until Indexer Has Coral",
          new IntakeUntilCoralInIndexer(wrist, intake, indexer, led));

      NamedCommands.registerCommand(
          "Finish Coral Intake", new IntakeFromIndexer(wrist, intake, indexer, led).withTimeout(2));

      NamedCommands.registerCommand(
          "Fix Coral Placement", new FixCoralPlacement(intake).withTimeout(1.5));
    }

    NamedCommands.registerCommand("Score Coral", new ScoreCoral(intake).withTimeout(0.25));

    NamedCommands.registerCommand(
        "Align Reef Left", new AlignReef(drive, vision, led, elevator, true));

    NamedCommands.registerCommand(
        "Align Reef Right", new AlignReef(drive, vision, led, elevator, false));

    NamedCommands.registerCommand(
        "Align Reef Left North", new AlignReefSetAngle(drive, vision, led, elevator, true, 21));

    NamedCommands.registerCommand(
        "Align Reef Left NorthWest", new AlignReefSetAngle(drive, vision, led, elevator, true, 20));

    NamedCommands.registerCommand(
        "Align Reef Left NorthEast", new AlignReefSetAngle(drive, vision, led, elevator, true, 22));

    NamedCommands.registerCommand(
        "Align Reef Left South", new AlignReefSetAngle(drive, vision, led, elevator, true, 18));

    NamedCommands.registerCommand(
        "Align Reef Left SouthWest", new AlignReefSetAngle(drive, vision, led, elevator, true, 19));

    NamedCommands.registerCommand(
        "Align Reef Left SouthEast", new AlignReefSetAngle(drive, vision, led, elevator, true, 17));

    NamedCommands.registerCommand(
        "Align Reef Right North", new AlignReefSetAngle(drive, vision, led, elevator, false, 21));

    NamedCommands.registerCommand(
        "Align Reef Right NorthWest",
        new AlignReefSetAngle(drive, vision, led, elevator, false, 20));

    NamedCommands.registerCommand(
        "Align Reef Right NorthEast",
        new AlignReefSetAngle(drive, vision, led, elevator, false, 22));

    NamedCommands.registerCommand(
        "Align Reef Right South", new AlignReefSetAngle(drive, vision, led, elevator, false, 18));

    NamedCommands.registerCommand(
        "Align Reef Right SouthWest",
        new AlignReefSetAngle(drive, vision, led, elevator, false, 19));

    NamedCommands.registerCommand(
        "Align Reef Right SouthEast",
        new AlignReefSetAngle(drive, vision, led, elevator, false, 17));

    NamedCommands.registerCommand("Stop Drivetrain", Commands.runOnce(() -> drive.stop(), drive));

    NamedCommands.registerCommand("Drive Backwards", new DriveBackwards(drive));

    NamedCommands.registerCommand("Align Source", new AlignSource(drive, false));
  }

  public Command getToggleBrakeCommand() {
    return Commands.runOnce(
            () -> {
              isBrakeMode = !isBrakeMode;
              drive.setBrakeMode(isBrakeMode);
              elevator.setBrakeMode(isBrakeMode);
              wrist.setBrakeMode(isBrakeMode);
              intake.setBrakeMode(isBrakeMode);
            })
        .ignoringDisable(true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
