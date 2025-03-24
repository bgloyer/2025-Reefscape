package frc.robot.subsystems.Drive.DriveAutomation;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveConstants;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.util.Helpers;

public class AlignToNet extends Command {
    private DriveSubsystem m_robotDrive;
    private ProfiledPIDController xController;
    private ProfiledPIDController turnController;

    public AlignToNet(DriveSubsystem subsystem) {
        m_robotDrive = subsystem;
        addRequirements(subsystem);
        xController = new ProfiledPIDController(DriveConstants.DriveToNetP, DriveConstants.DriveToNetI, DriveConstants.DriveToNetD, new Constraints(1, 1));
        xController.setTolerance(0.01);
        turnController = new ProfiledPIDController(DriveConstants.TurnkP, DriveConstants.TurnkI, DriveConstants.TurnkD, new Constraints(DriveConstants.TurnMaxVelocity, DriveConstants.TurnMaxAccel));
        turnController.enableContinuousInput(0, 360);
    }

    @Override
    public void initialize() {
        xController.reset(new State(m_robotDrive.getPose().getX(), m_robotDrive.getFieldRelSpeeds().vxMetersPerSecond));
        xController.setGoal(Helpers.isBlue ? AligningConstants.BlueNetXPosition : AligningConstants.RedNetXPosition);
        turnController.reset(new State(m_robotDrive.getHeading(), -m_robotDrive.getTurnRate()));
        turnController.setTolerance(5);
        turnController.setGoal(0);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double driveOutput = xController.calculate(m_robotDrive.getPose().getX()) * (Helpers.isBlue ? 1 : -1);
        double rotOutput = turnController.calculate(Helpers.betterModulus(m_robotDrive.getHeading(), 360));
        m_robotDrive.driveWithController(driveOutput, rotOutput, true);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_robotDrive.driveWithController(0, 0, true);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return xController.atGoal() && turnController.atGoal();
    }
}
