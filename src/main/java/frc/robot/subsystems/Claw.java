package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ClawConstants.CoralIntakeConstants;
import frc.robot.constants.ClawConstants.WristConstants;
import frc.robot.constants.Configs.ClawConfig;

public class Claw extends SubsystemBase {
    
    private final SparkFlex m_wristMotor;
    private final SparkFlex m_intakeMotor;
    private final SparkClosedLoopController m_wristController;
    private final SparkClosedLoopController m_intakeController;
    private final RelativeEncoder m_encoder; 
    private double targetAngle;
    private double currentSetpoint;
    private final LaserCan m_laser;
    public boolean positioningCoral;

    public Claw() {
        m_wristMotor = new SparkFlex(WristConstants.MotarCanId, MotorType.kBrushless);
        m_wristMotor.configure(ClawConfig.wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_wristController = m_wristMotor.getClosedLoopController();
        m_encoder = m_wristMotor.getEncoder();
        m_encoder.setPosition(WristConstants.Initial);
        m_wristMotor.configure(ClawConfig.wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_intakeMotor = new SparkFlex(CoralIntakeConstants.MotorCanId, MotorType.kBrushless);
        m_intakeMotor.configure(ClawConfig.intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_intakeController = m_intakeMotor.getClosedLoopController();
        resetSetpoint();

        m_laser = new LaserCan(Constants.CoralLaserCanID);
        try {
            m_laser.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
        } catch (Exception e) {
            System.out.println("laser can is the worst");
        }
    }

    public void resetSetpoint() {
        currentSetpoint = getAngle();
        targetAngle = currentSetpoint;
    }

    public double getAngle() {
        return m_encoder.getPosition();
    }

    public void runIntake() {
        runVoltage(CoralIntakeConstants.IntakeVoltage);
    }

    public void runOuttake() {
        runVoltage(CoralIntakeConstants.OuttakeVoltage);
    }

    public void runVoltage(double volts) {
        m_intakeController.setReference(volts, ControlType.kVoltage);
    }

    public void stopIntake() {
        m_intakeMotor.stopMotor();
    }

    public void setTargetAngle(double angle) {
        targetAngle = MathUtil.clamp(angle, WristConstants.MinAngle, WristConstants.MaxAngle); 
        currentSetpoint = getAngle();
    }

    private void positionSlewRateLimiting() {
        double error = targetAngle - currentSetpoint;
        currentSetpoint += Math.min(Math.abs(error), WristConstants.SlewRate) * Math.signum(error);
        m_wristController.setReference(currentSetpoint, ControlType.kPosition);
    }

    public boolean onTarget() {
        return Math.abs(m_encoder.getPosition() - targetAngle) < WristConstants.Tolerance;
    }

    public double getDistance() {
        if (m_laser.getMeasurement() == null)
            return 0;
        else
            return m_laser.getMeasurement().distance_mm;
    }

    public boolean coralStored() {
        return (m_laser.getMeasurement() != null) && (m_laser.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) && getDistance() < 100;
    }

    @Override
    public void periodic() {
        positionSlewRateLimiting();
        SmartDashboard.putNumber("Wrist Angle", getAngle());
        SmartDashboard.putNumber("Intake Velocity", m_intakeMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Coral Stored", coralStored());
    }
}
