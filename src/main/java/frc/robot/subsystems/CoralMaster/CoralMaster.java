package frc.robot.subsystems.CoralMaster;

import static frc.robot.util.Helpers.txToDistanceOffset;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Claw.ClawConstants.WristConstants;
import frc.robot.subsystems.Drive.VisionConstants;
import frc.robot.subsystems.Drive.DriveAutomation.AligningConstants;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.util.Level;

public class CoralMaster extends SubsystemBase {
    private final Arm m_arm;
    private final Elevator m_elevator;
    private final Claw m_claw;
    private Level level = Level.STORE;
    private boolean useIntakeAutoAlign = true;
        private boolean ranL4;
            
            public CoralMaster(Arm arm, Elevator elevator, Claw claw ) {
                m_arm = arm;
                m_elevator = elevator;
                m_claw = claw;
            }
        
            public Arm getArm() {
                return m_arm;
            }
        
            public Claw getClaw() {
                return m_claw;
            }
        
            public Elevator getElevator() {
                return m_elevator;
            }
        
            public void setState(double elevatorPosition, double armAngle, double clawangle) {
                m_elevator.setTarget(elevatorPosition);
                m_arm.setTargetAngle(armAngle);
                m_claw.setTargetAngle(clawangle);
            }
        
            public void setState(double elevatorPosition, double clawangle) {
                m_elevator.setTarget(elevatorPosition);
                m_claw.setTargetAngle(clawangle);
            }
        
            public void setState(Level level) {
                m_elevator.setTarget(level.elevatorHeight);
                m_arm.setTargetAngle(level.armAngle);
                m_claw.setTargetAngle(level.wristAngle);
            }
        
            public void setIntake() {
                setState(ElevatorConstants.Station, ArmConstants.Station, WristConstants.Station); 
                m_claw.runIntake();  
            }
    
            public void setOneCoralAwayIntake() {
                setState(ElevatorConstants.StationOffset, ArmConstants.StationOffset, WristConstants.StationOffset); 
                m_claw.runIntake();  
            }
        
            public void setStore() {
                setState(ElevatorConstants.Store, ArmConstants.Store, WristConstants.Store);
            }
        
            public void runIntake() {
                m_claw.runIntake();
            }
        
            public void runOuttake() {
                m_claw.runOuttake();
            }
        
            public void stopIntake() {
                m_claw.stopIntake();
            }
        
            public boolean onTarget() {
                return m_arm.onTarget() && m_claw.onTarget() && m_elevator.onTarget();
            }
        
            public boolean coralStored() {
                return m_claw.backLaserTriggered();
            }
        
            public boolean readyToScore() {
                return onTarget() && level.isReefScoringPosition;
            }
        
            public void setCurrentLevel(Level level) {
                this.level = level;
            }
        
            public boolean scoringButNotDealg() {
                return level == Level.FOUR || level == Level.TWO || level == Level.THREE || level == Level.ONE;
            }
        
            public Level getLevel() {
                return level;
            }
        
            public boolean useIntakeAutoAlign() {
                return useIntakeAutoAlign;
            }
        
            public void toggleIntakeAutoAlign() {
                useIntakeAutoAlign = !useIntakeAutoAlign;
            }
    
            public void setranL4(boolean value) {
                ranL4 = value;
        }
            public boolean getRanL4() {
                return ranL4;
            }

            public boolean alignedToMiddle() {
                return MathUtil.isNear(AligningConstants.MiddleReefOffset, txToDistanceOffset(VisionConstants.ReefLightLightName), 0.05);
            }

        @Override
        public void periodic() {
            SmartDashboard.putBoolean("Enable Intake Auto Align", useIntakeAutoAlign);
        }
    
        public void setLevelWithCoralOffset() {
            switch (level) {
                case TWO:
                    setState(Level.OFFSETTWO);
                    
                    break;
                case THREE:
                    setState(Level.OFFSETTHREE);
                    break;
                case FOUR:
                    setState(Level.OFFSETFOUR);
                    break;
                default:
                    break;
            }
        }
}

