/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;


import static frc.robot.Constants.PhotonVision.April_Tag_Back_pos;

import static frc.robot.Constants.PhotonVision.April_Tag_Front_pos;
import static frc.robot.Constants.PhotonVision.Camera_Name_Back;

import static frc.robot.Constants.PhotonVision.camera_Name_Front;
import static frc.robot.Constants.PhotonVision.tagLayout;

import java.util.Optional;

public final class VisionModule {

    private final PhotonCamera aprilTagsFront;
    private final PhotonCamera aprilTagsBack;
    private PhotonPoseEstimator photonEstimatorFront;
    private PhotonPoseEstimator photonEstimatorBack;

    private static VisionModule instance;

    private VisionModule() {
        PhotonCamera.setVersionCheckEnabled(false);

        aprilTagsFront = new PhotonCamera(camera_Name_Front);
        aprilTagsBack = new PhotonCamera(Camera_Name_Back);


        photonEstimatorBack =
                new PhotonPoseEstimator(
                        tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.PhotonVision.April_Tag_Back_pos);
        photonEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonEstimatorFront = 
                new PhotonPoseEstimator(
                        tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,Constants.PhotonVision.April_Tag_Front_pos);
        photonEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public static synchronized VisionModule getInstance() {
        if (instance == null) {
            instance = new VisionModule();
        }
        return instance;
    }

    public PhotonCamera getAprilTagsFrontRightCamera() {
        return aprilTagsFront;
    }

    public void setPhotonEstimatorFront(Transform3d transform3d) {
        this.photonEstimatorFront = new PhotonPoseEstimator(
                tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, transform3d);
        photonEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void setPhotonEstimatorBack(Transform3d transform3d) {
        this.photonEstimatorBack = new PhotonPoseEstimator(
                tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,transform3d);
        photonEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public PhotonCamera getAprilTagsRearLeftCamera() {
        return aprilTagsBack;
    }

    public PhotonPoseEstimator getPhotonEstimatorFrontRight() {
        return photonEstimatorFront;
    }

    public PhotonPoseEstimator getPhotonEstimatorRearLeft() {
        return photonEstimatorBack;
    }

    public void trackPipelineResults() {
    }

}