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

import au.grapplerobotics.CanBridge;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.Comparator;
import java.util.Optional;
import org.jmhsrobotics.frc2025.subsystems.vision.VisionConstants;
import org.jmhsrobotics.frc2025.util.ControllerMonitor;
import org.jmhsrobotics.frc2025.util.Elastic;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    CanBridge.runTCP();
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Initialize URCL
    Logger.registerURCL(URCL.startExternal(Constants.CAN.kCanDeviceMap));

    // Start AdvantageKit logger
    Logger.start();
    SmartDashboard.putString("logsavefile", "");
    SmartDashboard.putData(
        "saveLog",
        new InstantCommand(
                () -> {
                  Logger.end();
                  Logger.end();
                  var path = Paths.get(RobotBase.isSimulation() ? "logs" : "/U/logs");
                  try {
                    Optional<Path> lastModifiedFile =
                        Files.list(path)
                            .filter(Files::isRegularFile)
                            .max(Comparator.comparingLong(f -> f.toFile().lastModified()));

                    lastModifiedFile.ifPresentOrElse(
                        file -> {
                          try {
                            var filename = SmartDashboard.getString("logsavefile", "unamed");
                            Files.move(
                                file,
                                file.getParent().resolve(filename + " " + file.getFileName()),
                                StandardCopyOption.REPLACE_EXISTING);
                          } catch (IOException e) {
                            e.printStackTrace();
                          }
                        },
                        () -> System.out.println("No files found in the directory."));
                  } catch (IOException e) {
                    e.printStackTrace();
                  }
                  // Logger.addDataReceiver(new WPILOGWriter());
                  // Logger.addDataReceiver(new NT4Publisher());
                  // For some reason starting the logger again crashes so just kill the code anway
                  // and let it restart. XD
                  System.exit(0);
                })
            .ignoringDisable(true));

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {

    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
    updateAscopeVis();
  }

  private void updateAscopeVis() {
    double height = robotContainer.elevator.getHeight();
    double gripperDegrees = robotContainer.wrist.getPositionDegrees();
    double climberDegrees = robotContainer.climber.getClimberPositionDegrees();
    double indexerDegrees = robotContainer.indexer.getPositionDegrees();
    Logger.recordOutput(
        "stage1",
        new Pose3d(new Translation3d(0, 0, height / 2), new Rotation3d(Rotation2d.fromDegrees(0))));
    Logger.recordOutput(
        "stage2",
        new Pose3d(new Translation3d(0, 0, height), new Rotation3d(Rotation2d.fromDegrees(0))));
    Logger.recordOutput(
        "gripper",
        new Pose3d(
            new Translation3d(0.2730451486, 0, 0.4064 + height),
            new Rotation3d(0, Units.degreesToRadians(gripperDegrees), 0)));
    Logger.recordOutput(
        "climber",
        new Pose3d(
            new Translation3d(-.075, 0.267, 0.165),
            new Rotation3d(Units.degreesToRadians(climberDegrees), 0, 0)));
    Logger.recordOutput(
        "indexer",
        new Pose3d(
            new Translation3d(-0.0125, 0, 0.9775),
            new Rotation3d(0, Units.degreesToRadians(indexerDegrees), 0)));
    var robotpos = new Pose3d(robotContainer.drive.getPose());
    Logger.recordOutput(
        "camtest",
        robotpos.plus(
            new Transform3d(
                0.16,
                -0.29198,
                0.192,
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-10),
                    Units.degreesToRadians(35)))));
    // VisionConstants.robotToCamera1 =
    //     new Transform3d(
    //         Units.inchesToMeters(8),
    //         Units.inchesToMeters(-11.5),
    //         0.185,
    //         new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(35)));
    // VisionConstants.robotToCamera0 =
    //     new Transform3d(
    //         Units.inchesToMeters(8),
    //         Units.inchesToMeters(11.5),
    //         0.185,
    //         new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(-35)));
    Logger.recordOutput(
        "cam/left_robot_different", new Pose3d().plus(VisionConstants.robotToCamera0));
    Logger.recordOutput(
        "cam/right_robot_different", new Pose3d().plus(VisionConstants.robotToCamera1));
    Logger.recordOutput("cam/left_field", robotpos.plus(VisionConstants.robotToCamera0));
    Logger.recordOutput("cam/right_field", robotpos.plus(VisionConstants.robotToCamera1));
    if (robotContainer.intake.isCoralInIntake()) {
      Logger.recordOutput(
          "hasPipe", robotpos.plus(new Transform3d(0.0, 0.0, 1.2, new Rotation3d())));
    } else {
      Logger.recordOutput("hasPipe", robotpos.plus(new Transform3d(90, 0.0, 4, new Rotation3d())));
    }
    if (robotContainer.intake.isAlgaeInintake()) {
      Logger.recordOutput(
          "hasball", robotpos.plus(new Transform3d(0.0, 0.0, 1.5, new Rotation3d())));
    } else {
      Logger.recordOutput("hasball", robotpos.plus(new Transform3d(90, 0.0, 4, new Rotation3d())));
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    ControllerMonitor.checkController();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Elastic.selectTab("Autonomous");
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    Elastic.selectTab("Teleoperated");
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    Elastic.selectTab("Tuning");
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setEnabled(true);
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
