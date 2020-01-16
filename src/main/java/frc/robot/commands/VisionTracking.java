package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;


public class VisionTracking extends CommandBase {

    NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    double tv = get("tv");
    double previousError, previousZoom = 0;
    //public double tar;

    public double get(String var) {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry(var).getDouble(0.0);
    }

    public VisionTracking() {
    }

    public void setTracking(boolean tracking) {
        m_limelightTable.getEntry("camMode").setNumber(0);
        m_limelightTable.getEntry("ledMode").setNumber(0);
    }

    
    public void initialize() {
    }

    public void execute() {
    }

    public double pidX() {
        double min = 0.05;
        double kP = -0.04;// -0.038;
        double tx = get("tx");
        double error = -tx;

        if (tx > 0.0) {
            return kP * error - min;
        } else {
            return kP * error + min;
        }

    }

    public double getTarget() {
        double _minhor = 38, _minvert = 17;
        double  _maxhor = 195, _maxvert = 65;
        double whratio = get("thor") / get("tvert");
        double minratio = _minhor / _minvert;
        double maxratio = _maxhor / _maxvert;

        SmartDashboard.putNumber("minratio", minratio);
        SmartDashboard.putNumber("maxratio", maxratio);

        if (get("tv") == 1) {
            if (minratio < whratio && whratio < maxratio) {
                return 1;
            } else {
                return 0; 
            }
        } else {
            return 0;
        }
    }

    public double zoomForward() {
        if (getTarget() == 0) {
            return previousZoom;
        } else {
            double lmin = .5;
            double dkP = 0.1;
            double currentta = get("ta");
            double idealta = 7;
            double error = idealta - currentta;
            return previousZoom = Math.max(dkP * error, lmin);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}