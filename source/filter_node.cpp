#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "filter.hpp"

class FilterNode {

private:

    // TF related variables
    // ------------------------------
    tf2_ros::Buffer 
        tf_buffer_;
    
    tf2_ros::TransformListener 
        tf_listener_;

    tf2::Quaternion 
        rotation_;                   // The last orientation received by the imu - already in the frame_id coordinates

    double
        last_altitude_;                 // The last altitude received - already in the frame_id coordinates

    tf2::Transform
        tf_base_link_odom_;             // The transform of the base_link relative to the odom frame (arrow points from base_link to odom)

    std::string 
        frame_id_;                      // The working frame - "base_link"

    bool 
        flag_received_odom_,            // Indicates whether the first odometry message was received or not
        param_publish_tf_;

    // ------------------------------

    // Filter related variables
    // ------------------------------
    bool
        flag_received_imu_,
        flag_received_altitude_,        // Indicates whether the first altitude message was received (true) or not (false)
        flag_updated_weights_,
        param_direct_resample_,         // If set to true, the particles are resampled right after 
                                        //    each weight update.
        param_use_message_covariance_;  // If set to true, the covariance received is used for both 
                                        //    prediction and update. Otherwise the covariance is set 
                                        //    via parameter server.

    MapInterface map_;              // Grid-like map that stores the altitudes.

    int 
        nparticles_;                // The number of particles used for state estimation.

    RelativeAltitudeFilter 
        * filter_ = 0;              // Instance of the particle filter.

    std::map < std::string , std::map<std::string, tf::StampedTransform> > transforms_;

    double
        param_resample_rate_,       // Frequency the filter resamples particles
        stdev_odom_x_,              // The standard deviation on the x axis of each particle during prediction. 
                                    //    Only used when `param_use_message_covariance_` is false.
        stdev_odom_y_,              // The standard deviation on the y axis of each particle during prediction. 
                                    //    Only used when `param_use_message_covariance_` is false.
        stdev_orientation_,         // The standard deviation on the yaw of each particle during prediction 
                                    //    and IMU orientation correction. Only used when 
                                    //    `param_use_message_covariance_` is false.
        stdev_meas_altitude_;       // The standard deviation on the altitude measurement of each particle 
                                    //    during update. Only used when `param_use_message_covariance_` is false.

    // ------------------------------

    // ROS
    // ------------------------------

    ros::Publisher
        publisher_particles_,       // Publishes a PointCloud2 message of the particles' state.
        publisher_pose_;            // Publiches the Pose of the robot for debugging purposes
        
    ros::Subscriber
        subscriber_odom_,           // Receives an Odometry message used for filter prediction.
        subscriber_altitude_,       // Receives a PoseWithCovarianceStamped message that contains the altitude 
                                    //    measured.
        subscriber_imu_;            // Receives an Imu message from which the orientation is used.  
    
    image_transport::Publisher
        publisher_map_;             // Publishes an Image of the altitude map and the particles scattered over it.

    // ------------------------------

public:

    // Node
    // ------------------------------------------------------------------------

    FilterNode(ros::NodeHandle & node_handle) : 
        rotation_(0,0,0,1),
        last_altitude_(0),
        flag_received_imu_(false),
        flag_updated_weights_(false),
        flag_received_odom_(false),
        frame_id_("base_link"), 
        tf_listener_(tf_buffer_) 
    {

        // Local vars
        std::string 
            map_path, 
            sub_odom_topic, 
            sub_altitude_topic,
            sub_imu_topic;

        // Load parameters
        ros::param::param<int>        ("~nparticles",             nparticles_,            1000);
        ros::param::get               ("~map_path",               map_path);
        ros::param::param<bool>       ("~publish_tf",             param_publish_tf_,      false);
        ros::param::param<bool>       ("~direct_resample",        param_direct_resample_, false); 
        ros::param::param<double>     ("~resample_rate",          param_resample_rate_, 1.0/3.0); 
        ros::param::param<bool>       ("~use_message_covariance", param_use_message_covariance_,  true); 
        ros::param::param<std::string>("~sub_odom_topic",         sub_odom_topic,         "odom");
        ros::param::param<std::string>("~sub_altitude_topic",     sub_altitude_topic,     "sensor/altitude/data");
        ros::param::param<std::string>("~sub_imu_topic",          sub_imu_topic,          "sensor/imu/data");
        if(!param_use_message_covariance_) {
            ros::param::get("~stdev_odom_x",        stdev_odom_x_); 
            ros::param::get("~stdev_odom_y",        stdev_odom_y_); 
            ros::param::get("~stdev_orientation",   stdev_orientation_); 
            ros::param::get("~stdev_meas_altitude", stdev_meas_altitude_); 
        }

        // Start particle filter
        map_.load(map_path);
        filter_ = new RelativeAltitudeFilter(map_);
        ROS_INFO("[FilterNode] Loaded map.");
        filter_->sampleUniform(nparticles_);
        ROS_INFO("[FilterNode] Sampled particles.");

        // Start publishers
        publisher_particles_ = node_handle.advertise<sensor_msgs::PointCloud2>("filter/particles", 2);
        publisher_pose_ = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("filter/pose", 2);
        image_transport::ImageTransport image_transport(node_handle);
        this->publisher_map_  = image_transport.advertise("filter/map", 3);
        ROS_INFO("[FilterNode] Assigned publishers.");

        // Start subscribers
        subscriber_odom_        = node_handle.subscribe(sub_odom_topic,      3, &FilterNode::odomCallback,this);  
        subscriber_altitude_    = node_handle.subscribe(sub_altitude_topic,  1, &FilterNode::altitudeCallback,this);
        subscriber_imu_         = node_handle.subscribe(sub_imu_topic,       1, &FilterNode::imuCallback,this);
        ROS_INFO("[FilterNode] Assigned subscribers.");

        return ;
    }

    void spin() {
        int 
            hz = 300;
        float
            resample_period = 1./(1.0 * param_resample_rate_);

        ros::Time 
            curr_time = ros::Time::now(),
            time_last_publish_particles = curr_time,
            time_last_publish_tf = curr_time,
            time_last_publish_map = curr_time,
            time_last_resample = curr_time;

        ros::Rate loop_rate(hz);
        while(ros::ok()){
            ros::spinOnce();
            curr_time = ros::Time::now();
            double secs = 0;
            // Resample if not resampled directly from node and if weights were updated
            if(!param_direct_resample_) {
                secs = (curr_time - time_last_resample).toSec();
                if( secs >= resample_period && flag_updated_weights_) {
                    this->resample();
                    flag_updated_weights_ = false;
                    time_last_resample = curr_time;
                }
            }
            // Publish topics
            secs = (curr_time - time_last_publish_particles).toSec();
            if(secs >= 1.0/60.0 && publisher_particles_.getNumSubscribers() != 0) { 
                this->publishParticles();
                time_last_publish_particles = curr_time;
            }
            secs = (curr_time - time_last_publish_tf).toSec();
            if(secs >= 1e-2 && param_publish_tf_ == true) { 
                this->publishTF();
                time_last_publish_tf = curr_time;
            }
            secs = (curr_time - time_last_publish_map).toSec();
            if(secs >= 0.1 && publisher_map_.getNumSubscribers() != 0) { 
                this->publishMap();
                time_last_publish_map = curr_time;
            }

            // Delay
            loop_rate.sleep();
        }
        return ;
    }

    // Compute the transform from "map" to the working frame
    inline tf2::Transform getEstimatedTransformFrameRelativeToMap() const {
        tf2::Transform transform;
        double 
            x = 0, 
            y = 0, 
            z = 0,
            norm_factor = 0;
            //norm_factor = nparticles_;

        const std::vector<Particle> & particles = filter_->getParticles();
        for ( const Particle & p : particles ) {
            x += p.x * p.w;
            y += p.y * p.w;
            norm_factor += p.w;
        }
        x /= norm_factor;
        y /= norm_factor;
        z = map_.at((float)x,(float)y);

        // TODO: Consider computed yaw for rotation -> if no IMU is received this will help
        transform.setOrigin( tf2::Vector3(x, y, z) );
        transform.setRotation(rotation_);
        return transform;
    }

    inline void resample() {
        // Resample particles
        filter_->resample();
        return ;
    }

    // ------------------------------------------------------------------------

    // Callbacks
    // ------------------------------------------------------------------------

    //! Predicts the new position of the particles based on the movement w.r.t. to odom
    //! 
    //! @param msg: the odometry of frame_id_ ("base_link") w.r.t. to "odom"
    //! 
    void odomCallback(
        const nav_msgs::Odometry::ConstPtr & msg
    ) {
        //ROS_INFO("---- STARTED ODOM ----");
        tf2::Vector3 translation_msg(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );
        tf2::Quaternion quat_msg(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Transform tf_curr(quat_msg, translation_msg);

        float Delta_x = 0, Delta_y = 0;
        double Delta_row, Delta_pitch, Delta_yaw;
        if(flag_received_odom_) {
            const tf2::Transform & tf_prev = tf_base_link_odom_; // The transform of the base_link relative to the odom frame
            tf2::Transform tf_relative = tf_prev.inverseTimes(tf_curr); // Since odom retrieves the transform that takes a point from the base_link to the odom frame
                                                                        //  we want the "arrow" from t_curr to t_prev
            tf2::Vector3 translation = tf_relative.getOrigin();
            Delta_x = translation.x();
            Delta_y = translation.y();
            tf2::Quaternion q = tf_relative.getRotation();
            tf2::Matrix3x3 r_matrix(q);
            r_matrix.getRPY(Delta_row, Delta_pitch, Delta_yaw);
        }
        else {
            const geometry_msgs::Pose & pose = msg->pose.pose;
            Delta_x = pose.position.x;
            Delta_y = pose.position.y;
            tf2::Quaternion q(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            );
            tf2::Matrix3x3 r_matrix(q);
            r_matrix.getRPY(Delta_row, Delta_pitch, Delta_yaw);
        }

        // Predict
        float stdev_x = (param_use_message_covariance_) ? std::sqrt(msg->pose.covariance[0]) : stdev_odom_x_;
        float stdev_y = (param_use_message_covariance_) ? std::sqrt(msg->pose.covariance[1 * 6 + 1]) : stdev_odom_y_; // row * ncol + col
        float stdev_yaw = (param_use_message_covariance_) ? std::sqrt(msg->pose.covariance[5 * 6 + 5]) : stdev_orientation_; // row * ncol + col

        filter_->predict(
            Delta_x, Delta_y, Delta_yaw, 
            stdev_x, stdev_y, stdev_yaw
        );
        flag_received_odom_ = true;
        tf_base_link_odom_ = tf_curr;
        //ROS_INFO("---- ENDED ODOM ----");
        return ;
    }

    //! Receives the altitude offset message (changes of the barometer w.r.t. the map),
    //! finds the altitude offset on the current working frame w.r.t. the map and increments particles'
    //! offsets.
    void altitudeCallback(
        const nav_msgs::Odometry::ConstPtr & msg
    ){
        //ROS_INFO("---- STARTED ALTITUDE ----");
        double altitude = 0;
        try{
            geometry_msgs::TransformStamped tf_stamped;

            // Where is the altimeter with respect to the map?
            tf_stamped = this->tf_buffer_.lookupTransform(
                frame_id_,                  // Target
                msg->header.frame_id,       // From
                ros::Time(0)                // When
            );
            tf2::Vector3 A_base(
                tf_stamped.transform.translation.x,
                tf_stamped.transform.translation.y,
                tf_stamped.transform.translation.z
            );
            tf2::Matrix3x3 R_base_map(rotation_);
            altitude = msg->pose.pose.position.z - (R_base_map * A_base).z();
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            return ;
        }
        if(!flag_received_altitude_) {
            flag_received_altitude_ = true;
            last_altitude_ = altitude;
            return ;
        }
        float delta_h = altitude - last_altitude_;
        float stdev = (param_use_message_covariance_) ? std::sqrt(msg->pose.covariance[35]) : stdev_meas_altitude_;
        last_altitude_ = altitude;
        flag_updated_weights_ = filter_->addAltitudeOffset(delta_h, stdev);
        if(param_direct_resample_) {
            this->resample();
        }
        //ROS_INFO("---- ENDED ALTITUDE ----");
        return ;
    }

    void imuCallback(
        const sensor_msgs::Imu::ConstPtr & msg
    ) {
        // Frame convertion
        tf2::Quaternion q_base_link_map(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        tf2::Matrix3x3 R_base_map;
        if(frame_id_.compare(msg->header.frame_id) != 0) {         
            try{
                geometry_msgs::TransformStamped tf_stamped;

                // Where is the base_link with respect to the imu?
                tf_stamped = this->tf_buffer_.lookupTransform(
                    msg->header.frame_id,       // Target
                    frame_id_,                  // From
                    ros::Time(0)                // When
                );

                tf2::Quaternion q_base_imu(
                    tf_stamped.transform.rotation.x,
                    tf_stamped.transform.rotation.y,
                    tf_stamped.transform.rotation.z,
                    tf_stamped.transform.rotation.w
                );
                tf2::Quaternion q_imu_map(
                    msg->orientation.x,
                    msg->orientation.y,
                    msg->orientation.z,
                    msg->orientation.w
                );
                tf2::Matrix3x3 
                    R_base_imu(q_base_imu),
                    R_imu_map(q_imu_map);
                R_base_map = R_imu_map * R_base_imu;
                R_base_map.getRotation(q_base_link_map);
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                return ;
            }
        }
        else {
            R_base_map.setRotation(q_base_link_map);
        }

        // Get yaw from rotation matrix to RPY
        double roll, pitch, yaw;
        R_base_map.getRPY(roll, pitch, yaw);
        yaw = (yaw < 0) ? yaw + 2 * M_PI : yaw;
        yaw = (yaw >= 2 * M_PI) ? yaw - 2 * M_PI : yaw;

        // Get deviation
        double stdev = (param_use_message_covariance_) ? std::sqrt(msg->orientation_covariance[8]) : stdev_orientation_; // Yaw standard deviation
        filter_->setOrientation(yaw, stdev);
        rotation_ = q_base_link_map;
        flag_received_imu_ = true;
        return ;
    }

    // ------------------------------------------------------------------------

    // Publishers
    // ------------------------------------------------------------------------

    void publishParticles() const{
        const std::vector<Particle> &particles = filter_->getParticles();

        sensor_msgs::PointCloud2 cloud;
        cloud.header.frame_id = "map";
        cloud.header.stamp = ros::Time::now();
        cloud.width  = particles.size();
        cloud.height = 1;
        cloud.is_bigendian = false;
        cloud.is_dense = true; // Does not accept invalid points

        //for fields setup
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(2,"xyz","rgba");
        modifier.resize(particles.size());

        //iterators
        sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<double> out_w(cloud, "rgba");

        for (const Particle & p : particles) {
            double z = map_.at(p.x, p.y);
            //store xyz and w in point cloud
            *out_x = p.x; *out_y = p.y;  *out_z = z;
            *out_w = p.w;

            //increment iterator
            ++out_x; ++out_y; ++out_z; ++out_w;
        }

        publisher_particles_.publish(cloud);
        return ;
    }

    void publishMap() const {
        std::size_t 
            nrows = map_.getRows(), ncols = map_.getCols();
        float
            min = map_.min(), max = map_.max(), delta = max-min;
        cv::Mat image(nrows, ncols, CV_8UC3);

        for (std::size_t row = 0; row < nrows; row++) {
            for (std::size_t col = 0; col < ncols; col++) {
                float z = map_.at(row,col);
                unsigned char color = (unsigned char) ( 255.0 * ((z-min)/(delta))  );
                image.at< cv::Vec3b >(row,col)[0] = color;
                image.at< cv::Vec3b >(row,col)[1] = color;
                image.at< cv::Vec3b >(row,col)[2] = color;
            }
        }

        const std::vector<Particle> & particles = filter_->getParticles();
        for(std::size_t idx = 0; idx < nparticles_ ; idx++) {
            int row = 0, col = 0;
            const Particle & p = particles[idx];
            map_.toGridPosition(p.x, p.y, row, col);
            cv::circle( 
                image,
                cv::Point(col, row),
                10,
                cv::Scalar( 0, 0, 255 ),
                -1,
                8
            );
        }
        cv::resize(image, image, cv::Size(), 255.0/(1.0 * nrows),255.0/(1.0 * ncols));

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        publisher_map_.publish(msg);
        return ;
    }

    static void debug(const tf2::Transform & tf_in) {
        double x, y, z;
        const tf2::Vector3 & translation = tf_in.getOrigin();
        x = translation.x(); y = translation.y(); z = translation.z();
        const tf2::Quaternion & rotation = tf_in.getRotation();
        tf2::Matrix3x3 m(rotation);
        ROS_INFO("[%.3f , %.3f , %.3f , %.3f]",m[0][0], m[0][1], m[0][2], x);
        ROS_INFO("[%.3f , %.3f , %.3f , %.3f]",m[1][0], m[1][1], m[1][2], y);
        ROS_INFO("[%.3f , %.3f , %.3f , %.3f]",m[2][0], m[2][1], m[2][2], z);
        ROS_INFO("[%.3f , %.3f , %.3f , %.3f]",0.0,0.0,0.0,1.0);
        return ;
    }

    void publishTF() const {
        if(!flag_received_odom_ && !flag_received_imu_) {
            ROS_WARN("Did not receive any odometry and/or imu message");
            return ;
        }
        // Compute the transform from the map to the odom frame
        static tf2_ros::TransformBroadcaster 
            br;
        tf2::Transform 
            tf_base_map  = this->getEstimatedTransformFrameRelativeToMap();         // Transform of base_link relative to the map
        tf2::Vector3 translation = tf_base_map.getOrigin();
        const tf2::Quaternion & rotation = tf_base_map.getRotation();
        geometry_msgs::TransformStamped
            tf_msg;
        tf_msg.header.stamp = ros::Time(0);
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = translation.x();
        tf_msg.transform.translation.y = translation.y();
        tf_msg.transform.translation.z = translation.z();
        tf_msg.transform.rotation.x = rotation.x();
        tf_msg.transform.rotation.y = rotation.y();
        tf_msg.transform.rotation.z = rotation.z();
        tf_msg.transform.rotation.w = rotation.w();
        br.sendTransform(tf_msg);
        return ;
    }

    // ------------------------------------------------------------------------
    
};

int main(int ac, char * av[]) {
    ros::init(ac, av, "filter_node");
    ros::NodeHandle nh;
    FilterNode node(nh);
    node.spin();
    return 0;    
}