// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

/** Add your docs here. */
public class TargetCalcs {
public AprilTagFieldLayout Layout;


public AprilTag targeTag;

public Integer TargetID;

private Pose2d TCUPose2d;


public TargetCalcs(){

    try {
        Layout = new AprilTagFieldLayout(Constants.APRILTAG_FIELD_LAYOUT_PATH);
      } catch (IOException e) {
        Layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
      }
  


} 

public Pose2d GetApriltagePose(int TagID){


Optional<Pose3d> oAprilTagpos3d = Layout.getTagPose(TagID);

Pose2d AprilTagpos2d = new Pose2d();

if (oAprilTagpos3d.isPresent()){

 AprilTagpos2d = oAprilTagpos3d.get().toPose2d();
    
}

return AprilTagpos2d;
}


public double getDistTo_Tag(int TagId, Pose2d RobotPose2D){
 Pose2d TagPose = GetApriltagePose(TagId);

 Pose2d RoboPose = RobotPose2D;

 double distanceinMeters = RoboPose.getTranslation().getDistance(TagPose.getTranslation());

 double distFT = Units.metersToFeet(distanceinMeters);

 SmartDashboard.putNumber("Dist to Traget", distFT);

return distFT;

}



public Rotation2d AbsRotationToTag(  Pose2d RobotPose2D){
 Translation3d translation3d = new Translation3d(16.579342,5.547867999999999,1.4511020000000001 );
  Quaternion q = new Quaternion(6.123233995736766e-17,0.0,0.0,1.0);
  Rotation3d rotation3d = new Rotation3d(q) ;
  Pose3d tag = new Pose3d(translation3d,rotation3d);
  Pose2d TagPose = tag.toPose2d();
Pose2d RoboPose = RobotPose2D;
Translation2d targeTranslation2d = TagPose.getTranslation();
Translation2d relativeTranslation = targeTranslation2d.minus(RoboPose.getTranslation());
Rotation2d rotationtotarget = new Rotation2d(relativeTranslation.getX(),relativeTranslation.getY());
SmartDashboard.putNumber("Rotation angle", rotationtotarget.getDegrees());
SmartDashboard.putString("Tag Pose",TagPose.toString());

Optional<Alliance> ally = DriverStation.getAlliance();

Rotation2d offsetrotation;

 if (ally.get() == Alliance.Red){

 offsetrotation = new Rotation2d(Units.degreesToRadians(-5));

 }

 if (ally.get() == Alliance.Blue){

offsetrotation = new Rotation2d(Units.degreesToRadians(175));

}

else {
  offsetrotation = new Rotation2d(Units.degreesToRadians(175));
}

//System.out.println(ally.get());

    return rotationtotarget.plus(offsetrotation);
}


public double AbsDistToSpeaker(Pose2d RobotPose2D){
  Pose2d TagPose = GetApriltagePose(TargetID);
Pose2d RoboPose = RobotPose2D;
// Translation2d targeTranslation2d = TagPose.getTranslation();
// Translation2d relativeTranslation = targeTranslation2d.minus(RoboPose.getTranslation());
// Double dist = relativeTranslation.getDistance(relativeTranslation);

double Dx = Math.abs(RoboPose.getX() - TagPose.getX());
double Dy = Math.abs(RoboPose.getY() - TagPose.getY());

double dist = Math.sqrt((Math.pow(Dx, 2)+ Math.pow(Dy, 2)));

SmartDashboard.putNumber("Abs Dist to Target Meters", dist);

    return dist;
}

public double absbyPostSetpointSpeaker(){


  Pose2d RobotPose2D = TCUPose2d;

double dist = AbsDistToSpeaker(RobotPose2D);

Double ArmSetpoint = ((0.2403 * Math.pow(dist,4)) + (- 3.4518 * Math.pow(dist,3)) + (12.502 * Math.pow(dist,2)) + (18.124 * dist) + (-27.503))  ;         //y = 0.2403x4 - 3.4518x3 + 12.502x2 + 18.124x - 27.503

  return ArmSetpoint;

}




public void periodic(){}

public void SetSpeakerTargetID(){

 Optional<Alliance> ally = DriverStation.getAlliance();
if (ally.isPresent()) {
    if (ally.get() == Alliance.Red) {
        
TargetID = 4;

    }
    if (ally.get() == Alliance.Blue) {
        
      TargetID = 7;
    }
}
else {

  TargetID = -5;
    
}

}








 public void UpdateLocalPose(Pose2d RobotPose2d){


  TCUPose2d = RobotPose2d ; 


 }


}