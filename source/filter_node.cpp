#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
//#include <tf/transform_listener.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2/matrix3x3.h>

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

    //tf::TransformListener 
    //    tf_listener_;

    std::string 
        frame_id_;

    bool 
        param_publish_tf_;

    // ------------------------------

    // Filter related variables
    // ------------------------------
    bool
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

    FilterNode(ros::NodeHandle & node_handle) : tf_listener_(tf_buffer_) {
        // Local vars
        std::string 
            map_path, 
            sub_odom_topic, 
            sub_altitude_topic,
            sub_imu_topic;

        // Load parameters
        ros::param::param<int>        ("~nparticles",             nparticles_,            1000);
        ros::param::param<std::string>("~frame_id",               frame_id_,              "base_link");
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
        flag_updated_weights_ = false;
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
                    filter_->resample();
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

    // ------------------------------------------------------------------------

    // Callbacks
    // ------------------------------------------------------------------------

    inline void getTransformedDelta(
        const std::string &frame_to,
        const std::string &frame_from,
        const nav_msgs::Odometry::ConstPtr & msg,
        geometry_msgs::Pose & pose_target_frame
    ) const {
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped = this->tf_buffer_.lookupTransform(
            frame_to,    // To
            frame_from,       // From
            ros::Time(0)        // When
        );
        tf2::Quaternion q;
        tf2::fromMsg(transform_stamped.transform.rotation, q);
        tf2::Matrix3x3 R(q);
        tf2::Vector3 Delta_S(
            msg->pose.pose.position.x, 
            msg->pose.pose.position.y, 
            msg->pose.pose.position.z
        );
        tf2::Vector3 new_delta = R * Delta_S;
        pose_target_frame.position.x = new_delta.x();
        pose_target_frame.position.y = new_delta.y();
        pose_target_frame.position.z = new_delta.z();
        pose_target_frame.orientation.x = 0;
        pose_target_frame.orientation.y = 0;
        pose_target_frame.orientation.z = 0;
        pose_target_frame.orientation.w = 1;
        return ;
    }

    void odomCallback(
        const nav_msgs::Odometry::ConstPtr & msg
    ) {

        std::string msg_frame_id = msg->header.frame_id;

        // Frame convertion
        geometry_msgs::Pose pose_target_frame; // The pose in the robot's target frame
        pose_target_frame = msg->pose.pose; /*
        if(frame_id_.compare(msg_frame_id) != 0) {
            geometry_msgs::TransformStamped transformStamped;
            try{
                this->getTransformedDelta(this->frame_id_, msg_frame_id, msg, pose_target_frame);
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
            }
        }
        else {
            pose_target_frame = msg->pose.pose;
        }*/

        float Delta_x = pose_target_frame.position.x;
        float Delta_y = pose_target_frame.position.y;
        tf::Quaternion q(
            pose_target_frame.orientation.x,
            pose_target_frame.orientation.y,
            pose_target_frame.orientation.z,
            pose_target_frame.orientation.w
        );
        tf::Matrix3x3 r_matrix(q);
        double Delta_row, Delta_pitch, Delta_yaw;
        r_matrix.getRPY(Delta_row, Delta_pitch, Delta_yaw);

        // Predict
        float stdev_x = (param_use_message_covariance_) ? std::sqrt(msg->pose.covariance[0]) : stdev_odom_x_;
        float stdev_y = (param_use_message_covariance_) ? std::sqrt(msg->pose.covariance[1 * 6 + 1]) : stdev_odom_y_; // row * ncol + col
        float stdev_yaw = (param_use_message_covariance_) ? std::sqrt(msg->pose.covariance[5 * 6 + 5]) : stdev_orientation_; // row * ncol + col

        filter_->predict(
            Delta_x, Delta_y, Delta_yaw, 
            stdev_x, stdev_y, stdev_yaw
        );
        return ;
    }

    void altitudeCallback(
        const nav_msgs::Odometry::ConstPtr & msg
    ){
        std::string msg_frame_id = msg->header.frame_id;
        // Frame convertion
        geometry_msgs::Pose pose_target_frame; // The pose in the robot's target frame
        geometry_msgs::TransformStamped transformStamped;
        try{
            this->getTransformedDelta(this->frame_id_, msg_frame_id, msg, pose_target_frame);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
        }
        float delta_h = pose_target_frame.position.z;
        float stdev =  (param_use_message_covariance_) ? std::sqrt(msg->pose.covariance[35]) : stdev_meas_altitude_;

        flag_updated_weights_ = filter_->addAltitudeOffset(delta_h, stdev);
        if(param_direct_resample_) {
            filter_->resample();
        }
        return ;
    }

    void imuCallback(
        const sensor_msgs::Imu::ConstPtr & msg
    ) {
        std::string msg_frame_id = msg->header.frame_id;
        geometry_msgs::Pose pose_msg;
        pose_msg.orientation = msg->orientation;

        // Frame convertion
        geometry_msgs::Pose pose_target_frame; // The pose in the robot's target frame
        pose_target_frame = pose_msg;
        if(frame_id_.compare(msg_frame_id) != 0) {
            ROS_WARN("Imu frame_id is different than %s. We recommend using the imu_transformer_node from ROS for frame convertion.", this->frame_id_.c_str());
        }

        tf::Quaternion q(
            pose_target_frame.orientation.x,
            pose_target_frame.orientation.y,
            pose_target_frame.orientation.z,
            pose_target_frame.orientation.w
        );
        // Get yaw from converted quaternion to RPY
        tf::Matrix3x3 q_matrix(q);
        double roll, pitch, yaw;
        q_matrix.getRPY(roll, pitch, yaw);
        yaw = (yaw < 0) ? yaw + 2 * M_PI : yaw;
        yaw = (yaw >= 2 * M_PI) ? yaw - 2 * M_PI : yaw;

        // Get deviation
        double stdev = (param_use_message_covariance_) ? std::sqrt(msg->orientation_covariance[8]) : stdev_orientation_; // Yaw standard deviation
        filter_->setOrientation(yaw, stdev);
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

    void publishTF() const {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        double 
            x = 0, 
            y = 0, 
            z = 0,
            w = 0,
            yaw = 0,
            norm_factor = 1.0 / (1.0 * nparticles_);
        const std::vector<Particle> & particles = filter_->getParticles();
        for ( const Particle & p : particles ) {
            x += p.x;
            y += p.y;
            yaw += p.orientation;
            w += p.w;
        }
        x *= norm_factor;
        y *= norm_factor;
        w *= norm_factor;
        z = map_.at((float)x,(float)y);
        yaw *= norm_factor;

        transform.setOrigin( tf::Vector3(x, y, z) );
        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", frame_id_));

        geometry_msgs::PoseWithCovarianceStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.position.z = z;
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();
        publisher_pose_.publish(msg);

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