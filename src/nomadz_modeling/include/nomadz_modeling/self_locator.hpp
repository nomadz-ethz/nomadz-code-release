#pragma once

#include <vector>
#include <Eigen/Core>
#include <tuple>

#include "nomadz_configuration/game_settings.hpp"
#include "nomadz_configuration/io.hpp"
#include "nomadz_modeling/landmark_registrator.hpp"
#include "nomadz_modeling/pose_registrator.hpp"
#include "nomadz_modeling/line_registrator.hpp"
#include "self_locator_parameters.hpp"
#include "nomadz_modeling/particle.hpp"

namespace nomadz_modeling {
  /**
   * @brief Node for self-localization using UKF-based particle filter.
   *
   * This node manages self-localization using a UKF-based particle filter algorithm.
   * It integrates odometry and sensor measurements, updates robot pose estimates,
   * performs particle resampling, and manages the lifecycle of localization parameters.
   */
  class SelfLocator {
  public:
    explicit SelfLocator(self_locator_parameters::Params parameters);

    /**
     * Initializes particles for the particle filter.
     */
    void initParticles();

    /**
     * Initializes particles for the particle filter after being penalized.
     */
    void initParticlesFromPenalty();

    /**
     * Initializes particles for the particle filter if lost.
     */
    void initParticlesIfLost();

    /**
     * Initializes particles for the particle filter if falling.
     */
    void initParticlesIfFalling();

    /**
     * Integrates odometry information into the particle filter hypotheses.
     * @param odometry_offset The change in position and orientation.
     */
    void motionUpdate(const Eigen::Vector3f& odometry_offset);

    /**
     * Performs UKF measurement update step for all particles.
     * @param perceived_landmarks A vector of perceived landmark observations.
     * @param perceived_intersections A vector of perceived intersection observations.
     * @param perceived_feature_poses A vector of perceived feature pose observations.
     * @param perceived_lines A vector of perceived line observations.
     * @return boolean value that is true if anything was registered
     */

    bool sensorUpdate(const std::vector<PerceivedLandmark>& perceived_landmarks,
                      const std::vector<PerceivedIntersection>& perceived_intersections,
                      const std::vector<PerceivedFeaturePose>& perceived_feature_poses,
                      const std::vector<PerceivedLine>& perceived_lines);

    Eigen::Vector2f sensorUpdateLine(Particle& particle,
                                     const std::vector<PerceivedLine>& perceived_lines,
                                     std::vector<RegisteredLine>& lines);

    Eigen::Vector2f sensorUpdateLandmark(Particle& particle,
                                         const std::vector<PerceivedLandmark>& perceived_landmarks,
                                         const std::vector<PerceivedIntersection>& perceived_intersections,
                                         std::vector<RegisteredLandmark>& landmarks);

    Eigen::Vector2f sensorUpdateFeaturePose(Particle& particle,
                                            const std::vector<PerceivedFeaturePose>& perceived_feature_poses,
                                            std::vector<RegisteredRobotPose>& robot_poses);

    nomadz_core::Pose2D getBestPose() const { return robot_pose_; }

    // For visualizing particles only
    std::vector<nomadz_core::Pose2D> getParticlePoses() const {
      std::vector<nomadz_core::Pose2D> poses;
      poses.reserve(particles_.size());
      for (const Particle& particle : particles_) {
        poses.push_back(particle.getPose());
      }
      return poses;
    }
    std::vector<RegisteredLine> debug_registered_lines;
    std::vector<RegisteredLandmark> debug_registered_landmarks;

  private:
    const Eigen::Vector3f INIT_DEVIATION_{0.15F, 0.15F, 0.15F};
    const Eigen::Vector3f INIT_PENALTY_DEVIATION_{0.3F, 0.3F, 0.15F};
    const Eigen::Vector3f INIT_FALL_DEVIATION_{0.3F, 0.3F, 0.3F};
    const Eigen::Vector3f INIT_LOST_DEVIATION_{0.5F, 0.5F, 0.6F};
    const Eigen::Vector3f DEFAULT_DEVIATION_{0.3F, 0.3F, 0.3F};

    /**
     * Updates the estimated robot pose based on particle filter results.
     */
    void updateRobotPose();

    /**
     * Resamples particles based on their validity.
     */
    void resampling();

    static bool isOnCarpet(const nomadz_core::Pose2D& robot_pose);
    self_locator_parameters::Params parameters_;
    nomadz_configuration::GameSettings settings_ = nomadz_configuration::getGameSettings();

    std::vector<Particle> particles_;

    LandmarkRegistrator landmark_registrator_;
    PoseRegistrator pose_registrator_;
    LineRegistrator line_registrator_;

    std::unique_ptr<Particle> last_best_particle_;
    nomadz_core::Pose2D robot_pose_;
  };
} // namespace nomadz_modeling
