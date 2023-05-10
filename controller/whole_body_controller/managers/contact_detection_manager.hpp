#pragma once

/**
 * Contact Detection Manager
 *
 * Uses multiple modalities, namely:
 *   1) kinematics information (e.g., foot position w.r.t. torso)
 *   2) contact sensor information
 * to determine which foot/feet have touched down. This module then
 * updates the state provider with the estimated contact states.
 */


#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/draco_controller/draco_interface.hpp"
#include "controller/filter/digital_filters.hpp"

class ContactDetectionManager {
public:
    enum LimitsIdx {low = 0, high = 1};
    ContactDetectionManager(PinocchioRobotSystem *_robot);
    ~ContactDetectionManager();

    /***
     * Converts voltage measurement for force estimates via linear model approximation,
     * then filters the estimated forces.
     * @param contact_normal_voltage_raw Voltage readings corresponding to the normal
     * direction of the contact sensor
     */
    void UpdateForceMeasurements(const Eigen::Vector2d &contact_normal_voltage_raw);

    /**
     * Updates state provider states by checking contact sensor and
     * kinematic measurements, accoring to the flags specified in . Currently, the contact sensors have equal priority
     * over the kinematic information, i.e., if the contact sensors indicate a
     * contact change, the state provider is updated accordingly. If the contact
     * sensors do not indicate a change, the foot height is checked and the
     * state provider is updated, if the corresponding height contact is detected.
     * @param _sp State provider containing contact information for each foot in
     *      b_lf_contact and b_rf_contact
     * @param _expected_contact_height Expected height w.r.t. world on each foot
     *      in the order [left, right]
     * @return
     */
    bool UpdateContactStates(DracoStateProvider *_sp,
                             const Eigen::Vector2d &_expected_contact_height);

    /**
     * Getters
     */
    double GetLFootNormalForceRaw();
    double GetRFootNormalForceRaw();
    double GetFootNormalForceFilt(int _end_effector);
    bool HasContactSensorTouchdown(int _end_effector);
    bool HasHeelToeTouchdown(int _end_effector);

private:
    void _ConvertVoltageToForce(const Eigen::Vector2d &contact_sensor,
                                Eigen::Vector2d &contact_force_out);

    /**
     * Updates the b_contact_touchdown_ flag based on a Schmitt trigger. The trigger
     * looks at both the current _sp->b_{l,r}f_contact_ flag and the filtered forces
     * to determine if the values in b_contact_touchdown_ must change.
     * @param _sp   State provider containing b_lf_contact_ and b_rf_contact_ flags.
     */
    void _CheckContactThresholds(const DracoStateProvider *_sp);

    /**
     *
     * @param expected_height_difference
     */
    void _CheckSwingFootContact(const double expected_height_difference);

    void _UpdateSwingSide(DracoStateProvider *_sp);

protected:
    PinocchioRobotSystem *robot_;

    // kinematics-related properties
    bool b_use_foot_height_;
    std::map<int, Eigen::Vector2d> foot_corner_map_;
    int swing_leg_name_;
    int support_leg_name_;
    double foot_height_tol_;
    std::vector<bool> b_heel_toe_touchdown_;

    // contact sensing
    bool b_debug_only_log_;
    bool b_use_contact_sensor_;
    Eigen::Vector2d contact_forces_raw_;    // lfoot_contact, rfoot_contact
    Eigen::Vector2d contact_forces_filt_;   // lfoot_contact, rfoot_contact
    Eigen::Vector2d volt_to_force_map_;
    Eigen::Vector2d volt_bias_;
    Eigen::Vector2d schmitt_thresholds_;
    std::unique_ptr<ExponentialMovingAverageFilter> contact_sensor_filter_;
    std::vector<bool> b_contact_touchdown_;

    // wait tools before re-transitioning
    int wait_count_post_transition_;
    int count_post_transition_;

};