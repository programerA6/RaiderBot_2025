package frc.robot; // import basic WPILib comands

import com.pathplanner.lib.auto.NamedCommands; // Provides functions/variables/methods/classes for interacting with the Pathfinder GUI.

import edu.wpi.first.math.geometry.Pose2d; // handling of the robots position and orientation
import edu.wpi.first.math.geometry.Rotation2d; // handling of the robots rotation
import edu.wpi.first.math.geometry.Translation2d; // handling of the position of the robot excluding rotation.
import edu.wpi.first.wpilibj.DriverStation; // facilitates interaction with the driver station software.
import edu.wpi.first.wpilibj.Filesystem; // allows the program to find paths that were deployed directly to the RoboRIO
import edu.wpi.first.wpilibj.RobotBase; // handles main robot loop and lifecycle.
import edu.wpi.first.wpilibj2.command.Command; // Action handling.
import edu.wpi.first.wpilibj2.command.Commands; // Faciliates the excecution of multiple simultanious comands.
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; // Xbox controller interface.
import java.io.File; //Read, write, or check file properties.

import edu.wpi.first.wpilibj2.command.button.Trigger; // Identify a condition and complete an action based on that input.
import swervelib.SwerveInputStream; // Format/modify driver joystick inputs before handoff to swerve drivetrain.
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser; // dropdown menu to select autonomous mode before match initiation.
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // data transmision to smartdashboard software.

import frc.robot.commands.ClimberUpCommand; // extend climbing implement.
import frc.robot.Constants.ClawConstants; // holds fixed values for the claw mechanism.
// import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.miscConstants; //miscellaneous configuratoin values.
import frc.robot.commands.ClimberDownCommand; // retract climbing implement.
import frc.robot.subsystems.Claw.Claw; // control motors and sensors for claw mechanism.
import frc.robot.subsystems.Climber.ClimbSubsystem; // control motors/pistons for climbing mechanism.
import frc.robot.subsystems.Elevator.ElevatorSubsystem; // control motors, and all other equiptment for elevator subsystems.
import frc.robot.subsystems.Swerve.SwerveSubsystem; // swerve drivetrain management.
import frc.robot.subsystems.Swerve.Vision; // camera processing.
import frc.robot.subsystems.Elevator.ElevatorSubsystemSim; // simulation-specific elavator control.
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer // define the class that will contain all of the code for this file.
{

  final         CommandXboxController DriveController = new CommandXboxController(0); // Create an xbox controller object on port 0
  final         CommandXboxController OPController = new CommandXboxController(1); // Create an xbox controller object on port 1
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  public final ClimbSubsystem 
  m_climber = new ClimbSubsystem();
//======================Define Auton Modes===============
  /*
    create varaible with the type "Command" which is a 
    generic type given to all comands. This allows the specific type to be specified later.
  */
  private final Command Leave; 
  private final Command algae_Left;
  private final Command Elevator_Test;
  private final Command Middle_Coral;
  private final Command Left_Coral;
  private final Command Right_Coral;


  SendableChooser<Command> m_chooser; // Creates a menu in Shuffleboard/Smart Dashboard that facilitates the selection of auton modes. 

  //=======================================================

  private final Claw sub_claw; // Create a variable that can not be redefined with the type claw, with the name sub_claw.
  private final ElevatorSubsystem elevator; // create a variable that can not be redefined with the type elevator subsystem with the name elevator.
  private final ElevatorSubsystemSim elevatorsim; // create a variable that can not be redefined with the type elavator subystem with the name elavatorsim.


  /*
    This is where driveAngularVelocity is first assigned a value. Its value is the position of the X and Y values of the left joystick of the xbox controller.
    Since "final" was not declared, its value can be altered and its name can be re-assigned. The position of the joystick is modified by a deadband,
    which helps to combat stick drift and prefents acidental movement when the joysticks should be stationary. There are also modifiers that set limits
    on the speed that the robot can move, which are the ".scale<movement type>" statements. The SwerveInputStream type is a helper of the YAGSL library,
    and it provides a steady "stream" of data for the drive algorithm. 
  */

  SwerveInputStream /*<-- type*/ driveAngularVelocity /* <-- Variable name*/ = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                             // read the Y value of the left joystick, and tie that to the forward backward movement of the robot
                                                             () /* the -> symbol tells the code "after the command that comes before the symbol is
                                                              complete, run the command after the symbol"*/ -> DriveController.getLeftY() * -1,
                                                             // read the X value of the left joystick and tie that to the strafe (side to side movement)
                                                             () -> DriveController.getLeftX() * -1)
                                                            //.withControllerRotationAxis(() -> DriveController.getRawAxis(2))
                                                            .withControllerRotationAxis(DriveController::getRightX) // read the X axis of the right joystick and use that to control rotation
                                                            .deadband(miscConstants.DEADBAND) // apply the  deadband (Near zero input will be ignored) for the joysticks
                                                            .scaleTranslation(0.20) // set translation movement speed to 20% of maximum
                                                            .scaleRotation(0.15) // Set rotation speed to 15% maximum
                                                            .allianceRelativeControl(true); // Sets the frame of reference for the controls
  

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream /*<-- type*/ driveDirectAngle /* <-- Variable Name*/ = driveAngularVelocity.copy().withControllerHeadingAxis(DriveController::getLeftX,
                                                                                             DriveController::getLeftY) // Creates a copy of the controller data, but relative to a different marker
                                                           .headingWhile(true);
                                                           

  /*
    Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream /*<-- type */ driveRobotOriented /*<-- variable name */ = driveAngularVelocity.copy()/*<-- creates a copy of the driveAngularVelocity variable*/.robotRelative(false) 
                                                             .allianceRelativeControl(true); /* <-- modifies the copy of the driveAngualVelocity variable, setting it to alliance relative.
                                                             This seams to orient the robot based on markers defined as alliance specific.*/

  /*
    This creates a secondary driveangularVelocity variable, this time with the suffix "Keyboard" which, most likely, is used for the simulation.
    The translation scaler is set to 0.8, which means the robot will move side to side at 80% throttle. 
  */
  SwerveInputStream /* <-- type */ driveAngularVelocityKeyboard /*<-- variable name */ = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                      () -> -DriveController.getLeftY(),
                                                                      () -> -DriveController.getLeftX())
                                                                    .withControllerRotationAxis(() -> DriveController.getRawAxis(
                                                                        // () -> -driverJoystick.getRawAxis(1),
                                                                        // () -> -driverJoystick.getRawAxis(0))
                                                                    // .withControllerRotationAxis(() -> driverJoystick.getRawAxis(
                                                                        2))
                                                                    .deadband(miscConstants.DEADBAND)
                                                                    .scaleTranslation(0.8) //sets translation (side to side movement) throttle to 80% 
                                                                    .allianceRelativeControl(true); //sets the movement relative to the alliance markers. 
  // Derive the heading axis with math!
  SwerveInputStream /*<-- type */ driveDirectAngleKeyboard /*<-- variable name*/    = driveAngularVelocityKeyboard.copy() /*<-- create a copy of the driveAngularVelocityKeyboard variable*/
                                                                               .withControllerHeadingAxiontroller.getRawAxis(
                                                                                                                  // driverJoystick.getRawAxis(
                                                                                                                      2) *
                                                                                                                  (Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                DriveController.getRawAxis(
                                                                                                                  // driverJoystick.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               (2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    sub_claw = new Claw();
    elevator = new ElevatorSubsystem();
    elevatorsim = new ElevatorSubsystemSim(elevator);




    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(false);


//========================Auton_Stuff===================================================
//=======================Nammed_Commands================================================

// NamedCommands.registerCommand("Saftey", Commands.run(() -> {
//   sub_claw.goToSetpoint(ClawConstants.Wrist.WristPositions.Elevator_Threh.getPos()); 
// }));
// NamedCommands.registerCommand("Elevator_Return", Commands.run(() -> {
//   elevator.goToSetpoint(ElevatorConstants.ElevatorConfigs.Positions.BOTTOM.getPos());
// }));
// NamedCommands.registerCommand("Algae_Level_1", Commands.run(() -> {
//   elevator.goToSetpoint(ElevatorConstants.ElevatorConfigs.Positions.Algae_1.getPos());
// }));
// NamedCommands.registerCommand("Algae_Level_2", Commands.run(() -> {
//   elevator.goToSetpoint(ElevatorConstants.ElevatorConfigs.Positions.Algae_2.getPos());
// }));

NamedCommands.registerCommand("Algae_Intake", Commands.run(() -> {
  sub_claw.goToSetpoint(ClawConstants.Wrist.WristPositions.Algae_Drive.getPos());
}));
NamedCommands.registerCommand("Algae_Intake_Rollers", Commands.run(() -> {
  sub_claw.setRollerPower(9); 
}));
NamedCommands.registerCommand("Stop", Commands.run(() -> {
  sub_claw.setRollerPower(0); 
}));
NamedCommands.registerCommand("Spit", Commands.run(() -> {
  sub_claw.setRollerPower(-8);  
}));
NamedCommands.registerCommand("Auto_Tip", Commands.run(() -> {
  sub_claw.goToSetpoint(ClawConstants.Wrist.WristPositions.Auto.getPos());
}));
NamedCommands.registerCommand("Climb_Up", Commands.run(() -> {
  OPController.back().whileTrue(new ClimberUpCommand(m_climber));
}));


    Leave = drivebase.getAutonomousCommand("Leave");
    algae_Left = drivebase.getAutonomousCommand("algae_Left");
    Elevator_Test = drivebase.getAutonomousCommand("Elevator Test");
    Middle_Coral = drivebase.getAutonomousCommand("Middle_Coral");
    Right_Coral = drivebase.getAutonomousCommand("Right Coral");
    Left_Coral = drivebase.getAutonomousCommand("Left Coral");
    

    m_chooser = new SendableChooser<Command>();

    m_chooser.addOption("Leave", Leave);
    m_chooser.addOption("algae_Left", algae_Left);
    m_chooser.addOption("Elevator Test", Elevator_Test);
    m_chooser.addOption("Middle Coral", Middle_Coral);
    m_chooser.addOption("Left Coral", Left_Coral);
    m_chooser.addOption("Right Coral", Right_Coral);


    SmartDashboard.putData(m_chooser);

//======================================================================================

  }
  private void configureBindings()
  {
        

 Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

 Command driveFieldOrientedDirectAngleKeyboard  = drivebase.driveFieldOriented(driveDirectAngleKeyboard);


   if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      DriveController.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      DriveController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }


  
      
      //~~~~~~~~~~~~~~~~~~OPControler~~~~~~~~~~~~~~~~~~~~~~~~
      
      OPController.start().whileTrue(new ClimberUpCommand(m_climber));
      OPController.back().whileTrue(new ClimberDownCommand(m_climber));

      OPController.axisGreaterThan(2, 0.01).whileTrue(Commands.run(() -> {
        sub_claw.setRollerPower(((OPController.getRawAxis(2) * 0.85) * 12) * -1);
      }));
      OPController.axisGreaterThan(3, 0.01).whileTrue(Commands.run(() -> {
        sub_claw.setRollerPower((OPController.getRawAxis(3) * 0.85) * 12);
      }));

      OPController.axisLessThan(2, 0.01).and(OPController.axisLessThan(2,0.01)).whileTrue(Commands.run(() -> {
       sub_claw.setRollerPower(0);
      }).repeatedly());


      OPController.axisMagnitudeGreaterThan(1, 0.25).whileTrue(Commands.run(() -> {
        sub_claw.setWrist((OPController.getRawAxis(1) * (12 * 0.25)) * -1);
//        System.out.println("Wrist Bump");
      })).whileFalse(Commands.runOnce(() -> {
        sub_claw.goToSetpoint(sub_claw.getEncoderMeasurement());
      }));


      // OPController.povUp().onTrue(elevator.setHeightCommand(0.5));

      // OPController.povRight().whileTrue(Commands.run(() -> {
      //   elevator.goToSetpoint(ElevatorConstants.ElevatorConfigs.Positions.Algae_1.getPos());
      // }));
      // OPController.povUp().whileTrue(Commands.run(() -> {
      //   elevator.goToSetpoint(ElevatorConstants.ElevatorConfigs.Positions.Algae_2.getPos());
      // }));
      // OPController.povDown().whileTrue(Commands.run(() -> {
      //   sub_claw.goToSetpoint(ClawConstants.Wrist.WristPositions.Intake2.getPos());
      // }));

      // OPController.povLeft().whileTrue(Commands.run(() -> {
      //   elevator.goToSetpoint(ElevatorConstants.ElevatorConfigs.Positions.L4.getPos());
      // }));

      //Wrist Pos Control 
      // OPController.y().whileTrue(Commands.run(() -> {
      //   sub_claw.goToSetpoint(ClawConstants.Wrist.WristPositions.Home.getPos());
      //   elevator.goToSetpoint(ElevatorConstants.ElevatorConfigs.Positions.INTAKE.getPos());
      // }));

      OPController.x().whileTrue(Commands.run(() -> {

        sub_claw.goToSetpoint(ClawConstants.Wrist.WristPositions.L1_L2_Coral.getPos());
        
      }));

      // OPController.b().whileTrue(Commands.run(() -> {
      //   sub_claw.goToSetpoint(ClawConstants.Wrist.WristPositions.Floor.getPos());
      //   elevator.goToSetpoint(ElevatorConstants.ElevatorConfigs.Positions.FloorIntake.getPos());
      // }));

      OPController.a().whileTrue(Commands.run(() -> {
        sub_claw.goToSetpoint(ClawConstants.Wrist.WristPositions.Elevator_Threh.getPos());
      }));

      OPController.leftBumper().whileTrue(Commands.run(() -> {
        sub_claw.goToSetpoint(ClawConstants.Wrist.WristPositions.Algae_Drive.getPos());
      }));

      OPController.rightBumper().whileTrue(Commands.run(() -> {
        sub_claw.setRollerPower(-9);      
      }));

      //~~~~~~~~~~~~~~~~~~Drive Control~~~~~~~~~~~~~~~~~~~~~~~~
      DriveController.a().onTrue((Commands.runOnce(drivebase::zeroGyroWithAlliance)));

    


      // DriveController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));

      //Go to pos ?
       //NOTE - Vision needs rewrite before ANY testing. Odom does not work and will throw the robot into a wall!!!!
      // DriveController.x().whileTrue(
      //   drivebase.driveToPose(new Pose2d(new Translation2d(14.4, 4.0), Rotation2d.fromDegrees(180))));

      //This is our boost control Right Trigger
      DriveController.axisGreaterThan(3, 0.01).onChange(Commands.runOnce(() -> {
        driveAngularVelocity.scaleTranslation(DriveController.getRightTriggerAxis() + 0.35);
        driveAngularVelocity.scaleRotation((DriveController.getRightTriggerAxis() * miscConstants.RotationSpeedScale) + 0.25);
      }).repeatedly()).whileFalse(Commands.runOnce(() -> { 
        driveAngularVelocity.scaleTranslation(0.25);
        driveAngularVelocity.scaleRotation(0.15);
      }).repeatedly());

       }

    
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }
  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
