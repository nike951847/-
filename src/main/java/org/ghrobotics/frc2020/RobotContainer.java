package org.ghrobotics.frc2020;

import java.util.List;

import org.ghrobotics.frc2020.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class RobotContainer {
  private Drivetrain drive = new Drivetrain();

  public Command getAutonomousCommand() {
    TrajectoryConfig config = new TrajectoryConfig(
        0.5, 0.1);
    config.setKinematics(drive.getKinematics());

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(2, 0)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(8, 0, new Rotation2d(ee0)),
        // Pass config
        config
    );

    RamseteCommand command = new RamseteCommand(
        trajectory,
        drive::getPose,
        new RamseteController(2, 0.7),
        drive.getFeedforward(),
        drive.getKinematics(),
        drive::getSpeeds,
        drive.getLeftPIDController(),
        drive.getRightPIDController(),
        drive::setOutputVolts,
        drive
    );

    return command.andThen(() -> drive.setOutputVolts(0, 0));
  }

  public void reset() {
    drive.reset();
  }
}
