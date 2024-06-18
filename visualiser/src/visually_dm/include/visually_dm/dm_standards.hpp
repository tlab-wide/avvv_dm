/**
 * @author AmirInt <amir.intelli@gmail.com>
 * @brief Includes the standards present in the DM protocol
*/

#ifndef VISUALLY_DM__DM_STANDARDS_HPP_
#define VISUALLY_DM__DM_STANDARDS_HPP_

namespace dm_standards
{

/**
 * @brief The coefficient in degrees of the values for the following messages:
 * 1. dm_object_info_msgs/Latitude
 * 2. dm_object_info_msgs/Longitude
*/
constexpr double altitude_coef{ 0.01 };

/**
 * @brief The coefficient in degrees of the values for the following messages:
 * 1. dm_object_info_msgs/Latitude
 * 2. dm_object_info_msgs/Longitude
*/
constexpr double location_coef{ 1e-7 };

/**
 * @brief The coefficient in degrees of the values for the following messages:
 * 1. dm_object_info_msgs/WGS84AngleAccuracy
 * 2. dm_object_info_msgs/WGS84AngleValue
*/
constexpr double angle_coef{ 0.01 };

/**
 * @brief The coefficient in metres of the values for the following messages:
 * 1. dm_object_info_msgs/ObjectDimensionAccuracy
 * 1. dm_object_info_msgs/DimensionValue
*/
constexpr double dimension_coef{ 0.01 };

/**
 * @brief The coefficient in seconds of the values for the following messages:
 * 1. dm_signal_info_msgs/MaxTimeToChange
 * 1. dm_signal_info_msgs/MinTimeToChange
*/
constexpr double signal_time_coef{ 0.1 };

/**
 * @brief The default dimensions of detected vehicles, people and animals
*/
constexpr double vehicle_dim[3]{ 4.0, 2.2, 1.6 };
constexpr double person_dim[3]{ 0.6, 0.6, 1.8 };
constexpr double animal_dim[3]{ 1.0, 0.6, 0.7 };

}

#endif // DM_STANDARDS_HPP_