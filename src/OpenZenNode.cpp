
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

#include "ManagedThread.h"
#include <OpenZen.h>

#include <memory>
#include <string>

class OpenZenSensor
{
 public:
    OpenZenSensor() : private_nh("~"), 
        m_sensorThread( [](SensorThreadParams const& param) -> bool {

        const float cDegToRad = 3.1415926f/180.0f;
        const float cEarthG = 9.81f;
        const float cMicroToTelsa = 1e-6f;

        auto event = param.zenClient->waitForNextEvent();
        auto have_event = event.first;
        auto event_value = event.second;

    if (!have_event) {
        // empty event received, terminate
        return false;
    }

    if (!event_value.component.handle)
    {
        // not an event from a component
        switch (event_value.eventType)
        {
            case ZenSensorEvent_SensorDisconnected:
                ROS_INFO("OpenZen sensor disconnected");
                return false;
        }
    }

    if (event_value.component.handle == 1) {
        if (event_value.eventType == ZenImuEvent_Sample) {
            // IMU
            auto const& d = event_value.data.imuData;

            sensor_msgs::Imu imu_msg;
            sensor_msgs::MagneticField mag_msg;

            // We follow this ROS conventions
            // https://www.ros.org/reps/rep-0103.html
            // https://www.ros.org/reps/rep-0145.html

            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = param.frame_id;

            // Fill orientation quaternion
            imu_msg.orientation.w = d.q[0];
            imu_msg.orientation.x = -d.q[1];
            imu_msg.orientation.y = -d.q[2];
            imu_msg.orientation.z = -d.q[3];

            // Fill angular velocity data
            // - scale from deg/s to rad/s
            imu_msg.angular_velocity.x = d.g[0] * cDegToRad;
            imu_msg.angular_velocity.y = d.g[1] * cDegToRad;
            imu_msg.angular_velocity.z = d.g[2] * cDegToRad;

            // Fill linear acceleration data
            const float rosConversion = -1.0 * (!param.useLpmsAccelerationConvention) +
                1.0 * param.useLpmsAccelerationConvention;

            imu_msg.linear_acceleration.x = rosConversion * d.a[0] * cEarthG;
            imu_msg.linear_acceleration.y = rosConversion * d.a[1] * cEarthG;
            imu_msg.linear_acceleration.z = rosConversion * d.a[2] * cEarthG;

            mag_msg.header.stamp = imu_msg.header.stamp;
            mag_msg.header.frame_id = param.frame_id;

            // Units are microTesla in the LPMS library, Tesla in ROS.
            mag_msg.magnetic_field.x = d.b[0] * cMicroToTelsa;
            mag_msg.magnetic_field.y = d.b[1] * cMicroToTelsa;
            mag_msg.magnetic_field.z = d.b[2] * cMicroToTelsa;

            // Publish the messages
            param.imu_pub.publish(imu_msg);
            param.mag_pub.publish(mag_msg);
        }
    }
        
    return true;
        })
    {
        // Get node parameters
        private_nh.param<std::string>("sensor_name", m_sensorName, "");
        private_nh.param<std::string>("sensor_interface", m_sensorInterface, "LinuxDevice");
        private_nh.param<bool>("openzen_verbose", m_openzenVerbose, false);

        // In LP-Research sensor output, the linear acceleration measurement is pointing down (z-) when
        // the sensor is lying flat on the table. ROS convention is z+ pointing up in this case
        // By default, this ROS driver converts to the ROS convention. Set this flag to true to
        // use the LPMS convention
        private_nh.param<bool>("use_lpms_acceleration_convention", m_useLpmsAccelerationConvention, false);
        private_nh.param<std::string>("frame_id", frame_id, "imu");
        imu_pub = nh.advertise<sensor_msgs::Imu>("imu",1);
        mag_pub = nh.advertise<sensor_msgs::MagneticField>("mag",1);

        auto clientPair = zen::make_client();
        m_zenClient = std::unique_ptr<zen::ZenClient>(new zen::ZenClient(std::move(clientPair.second)));

        if (clientPair.first != ZenError_None) {
            ROS_ERROR("Cannot start OpenZen");
            return;
        }

        if (m_openzenVerbose) {
            ZenSetLogLevel(ZenLogLevel_Debug);
        } else {
            ZenSetLogLevel(ZenLogLevel_Off);
        }

        // no sensor name given, auto-disove
        if (m_sensorName.size() == 0) {
            ROS_INFO_STREAM("OpenZen sensors will be listed");
            ZenError listError = m_zenClient->listSensorsAsync();

            if (listError != ZenError_None) {
                ROS_ERROR("Cannot list sensors");
                return;
            }
            
            bool listingDone = false;
            bool firstSensorFound = false;
            ZenSensorDesc foundSens;

            while (listingDone == false) {
                const auto pair = m_zenClient->waitForNextEvent();
                const bool success = pair.first;
                auto& event = pair.second;
                if (!success)
                    break;

                if (!event.component.handle)
                {
                    switch (event.eventType)
                    {
                    case ZenSensorEvent_SensorFound:
                        if (!firstSensorFound) {
                            foundSens = event.data.sensorFound;
                            firstSensorFound = true;
                        }
                        ROS_INFO_STREAM("OpenZen sensor with name " << event.data.sensorFound.serialNumber << " on IO system found" <<
                            event.data.sensorFound.ioType);
                        break;

                    case ZenSensorEvent_SensorListingProgress:
                        if (event.data.sensorListingProgress.progress == 1.0f) {
                            listingDone = true;
                        }
                            
                        break;
                    }
                }
            }

            if (!firstSensorFound) {
                ROS_ERROR("No OpenZen sensors found");
                return;
            }

            ROS_INFO_STREAM("Connecting to found sensor " << foundSens.serialNumber << " on IO system " << foundSens.ioType);
            auto sensorObtainPair = m_zenClient->obtainSensor(foundSens);

            if (sensorObtainPair.first != ZenSensorInitError_None) {
                ROS_ERROR("Cannot connect to sensor found with discovery. Make sure you have the user rights to access serial devices.");
                return;
            }
            m_zenSensor = std::unique_ptr<zen::ZenSensor>( new zen::ZenSensor(std::move(sensorObtainPair.second)));
        } else {
            // directly connect to sensor
            ROS_INFO_STREAM("Connecting directly to sensor " << m_sensorName << " over interface " << m_sensorInterface);
            auto sensorObtainPair = m_zenClient->obtainSensorByName(m_sensorInterface, m_sensorName);

            if (sensorObtainPair.first != ZenSensorInitError_None) {
                ROS_ERROR("Cannot connect directly to sensor.  Make sure you have the user rights to access serial devices.");
                return;
            }
            m_zenSensor = std::unique_ptr<zen::ZenSensor>( new zen::ZenSensor(std::move(sensorObtainPair.second)));
        }
    }

    bool run(void)
    {
        if (!m_zenClient) {
            ROS_ERROR("OpenZen could not be started");
            return false;
        }

        if (!m_zenSensor) {
            ROS_ERROR("OpenZen sensor could not be connected");
            return false;
        }

        m_sensorThread.start( SensorThreadParams{
            m_zenClient.get(),
            frame_id,
            imu_pub,
            mag_pub,
            m_useLpmsAccelerationConvention
        } );

        ROS_INFO("Data streaming from sensor started");

        return true;
    }

 private:

    // Access to ROS node
    ros::NodeHandle nh, private_nh;
    ros::Timer updateTimer;
    ros::Publisher imu_pub, mag_pub;

    std::unique_ptr<zen::ZenClient> m_zenClient;
    std::unique_ptr<zen::ZenSensor> m_zenSensor;

    bool m_openzenVerbose;
    bool m_useLpmsAccelerationConvention;

    // Parameters
    std::string m_sensorName;
    std::string m_sensorInterface;
    int baudrate;
    std::string frame_id;

    struct SensorThreadParams {
        zen::ZenClient * zenClient;
        std::string frame_id;
        ros::Publisher & imu_pub;
        ros::Publisher & mag_pub;
        bool useLpmsAccelerationConvention;
    };

    ManagedThread<SensorThreadParams> m_sensorThread;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "openzen_sensor");
    ros::NodeHandle nh, private_nh;

    OpenZenSensor lpOpenZen;

    lpOpenZen.run();

    ros::spin();

    return 0;
}
