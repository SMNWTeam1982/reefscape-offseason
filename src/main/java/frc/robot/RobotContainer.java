// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem.ElevatorConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.swerve.ReefNavigation;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final PhotonVisionSubsystem visionSubsystem =
      new PhotonVisionSubsystem(
          Constants.VisionConstants.PHOTON_CAM_RELATIVE_TO_ROBOT, "limelight-front");

  private final DriveSubsystem driveSubsystem =
      new DriveSubsystem(
          () -> visionSubsystem.getLastVisionData(), () -> visionSubsystem.isDataFresh());

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final LoggedDashboardChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandJoystick operatorController =
      new CommandJoystick(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  @SuppressWarnings("unused")
  public RobotContainer() {
    Command testAuto =
        AutoBuilder.pathfindToPose(
            new Pose2d(5.0, 5.0, new Rotation2d()),
            new PathConstraints(
                0.5,
                0.5,
                Math.PI / 2.0, // quarter rotation per second
                Math.PI / 2.0));
    autoChooser =
        new LoggedDashboardChooser<>("Selected Auto Routine", AutoBuilder.buildAutoChooser());
    autoChooser.addOption("testAuto", testAuto);
    // Configure the trigger bindings
    configureDriverBindings();
    configureOperatorBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   *
   * <p>see the wpilib page on coordinates and joystick coordinates
   */
  private void configureDriverBindings() {
    var alliance = DriverStation.getAlliance();
    boolean onBlueSide = true; // default to blue mode
    if (alliance.isPresent()) {
      onBlueSide = alliance.get() == Alliance.Blue;
    }

    driveSubsystem.setDefaultCommand(
        driveSubsystem.driveFromDriversStation(
            () -> {
              return new ChassisSpeeds(
                  deadZone(driverController.getRightX()) * 2 * 0.2,
                  deadZone(driverController.getRightY()) * 2 * 0.2,
                  deadZone(driverController.getLeftX()) * Math.PI * 0.2); // -PI - PI radians per second (-180 - 180 degrees/sec)
            },
            onBlueSide));

    
    driverController.a() // automatically moves to the closest reef scoring pose
        .debounce(0.01)
        .whileTrue(
          driveSubsystem.moveToPose(
            ReefNavigation.getClosestScoringPose(driveSubsystem.getEstimatedPose())
          )
        );

    // resets heading when button is released
    driverController.y()
        .debounce(0.01)
        .onFalse(driveSubsystem.zeroEstimatedHeading(visionSubsystem));
  }

  private void configureOperatorBindings() {

    //driverController.x().whileTrue(elevatorSubsystem.moveToTargetHeight(ElevatorConstants.LEVEL_2_TARGET_HEIGHT)).onFalse(elevatorSubsystem.setIdle());
    driverController.x().whileTrue(climberSubsystem.moveClimberOut());
    driverController.b().whileTrue(climberSubsystem.moveClimberIn());
  }

  private double deadZone(double number) {
    if (Math.abs(number) < 0.05) {
      return 0;
    }
    return number;
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
