#include "swiftnav_piksi/piksi_driver.h"
#include <libsbp/system.h>
#include <libsbp/navigation.h>
#include <libsbp/observation.h>

#include <iomanip>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/TimeReference.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>

namespace swiftnav_piksi
{	
	template <int MSG_TYPE>
	void sbpCallback(u16 sender_id, u8 len, u8 msg[], void *context)
	{
		if ( context == NULL )
		{
			ROS_ERROR("Context is null. This should never happen.");
			return;
		}

		PiksiDriver*driver = (PiksiDriver*) context;
		driver->sbpCallback(MSG_TYPE, sender_id, len, msg);

		return;
	}



	PiksiDriver::PiksiDriver( const ros::NodeHandle &_nh,
		const ros::NodeHandle &_nh_priv,
		const std::string & _port,
		const std::string & _frame_id,
		u16 publish_raw_msg_mask,
		bool subscribe_observation_data) :
		nh( _nh ),
		nh_priv( _nh_priv ),
		port( _port ),
		frame_id( _frame_id ),
		piksid( -1 ),

		heartbeat_diag(nh, nh_priv, "ppiksi_time_diag"),
		llh_diag(nh, nh_priv, "ppiksi_llh_diag"),
		rtk_diag(nh, nh_priv, "ppiksi_rtk_diag"),

		min_llh_rate( 0.5 ),
		max_llh_rate( 50.0 ),
		min_rtk_rate( 0.5 ),
		max_rtk_rate( 50.0 ),
		min_heartbeat_rate( 0.5 ),
		max_heartbeat_rate( 50.0 ),

		llh_pub_freq( diagnostic_updater::FrequencyStatusParam(
			&min_llh_rate, &max_llh_rate, 0.1, 50 ) ),
		rtk_pub_freq( diagnostic_updater::FrequencyStatusParam( 
			&min_rtk_rate, &max_rtk_rate, 0.1, 50 ) ),
		heartbeat_pub_freq( diagnostic_updater::FrequencyStatusParam( 
			&min_rtk_rate, &max_rtk_rate, 0.1, 50 ) ),

		io_failure_count( 0 ),
		last_io_failure_count( 0 ),
		open_failure_count( 0 ),
		last_open_failure_count( 0 ),
		heartbeat_flags( 0 ),

		num_llh_satellites( 0 ),
		llh_status( 0 ),
		llh_lat( 0.0 ),
		llh_lon( 0.0 ),
		llh_height( 0.0 ),
		llh_h_accuracy( 0.0 ),
		hdop( 1.0 ),

		rtk_status( 0 ),
		num_rtk_satellites( 0 ),
		rtk_north( 0.0 ),
		rtk_east( 0.0 ),
		rtk_height( 0.0 ),
		rtk_h_accuracy( 0.04 ),     // 4cm
		rtk_vel_east( 0.0 ),
		rtk_vel_north( 0.0 ),
		rtk_vel_up( 0.0 ),

		spin_rate( 2000 ),      // call sbp_process this fast to avoid dropped msgs
		spin_thread( &PiksiDriver::spin, this ),

		publish_raw_msg_mask_( publish_raw_msg_mask ),
		subscribe_observation_data_( subscribe_observation_data )
	{
		cmd_lock.unlock( );
		heartbeat_diag.setHardwareID( "piksi heartbeat" );
		heartbeat_diag.add( heartbeat_pub_freq );

		llh_diag.setHardwareID( "piksi lat/lon" );
		llh_diag.add( llh_pub_freq );

		rtk_diag.setHardwareID( "piksi rtk" );
		rtk_diag.add( "Piksi Status", this, &PiksiDriver::DiagCB );
		rtk_diag.add( rtk_pub_freq );
	}

	PiksiDriver::~PiksiDriver( )
	{
		spin_thread.interrupt( );
		PIKSIClose( );
	}

	bool PiksiDriver::PIKSIOpen( )
	{
		boost::mutex::scoped_lock lock( cmd_lock );
		return PIKSIOpenNoLock( );
	}

	bool PiksiDriver::PIKSIOpenNoLock( )
	{
		if( piksid >= 0 )
			return true;

		piksid = piksi_open( port.c_str( ) );

		if( piksid < 0 )
		{
			open_failure_count++;
			return false;
		}

		sbp_state_init(&state);
		sbp_state_set_io_context(&state, &piksid);

		sbp_register_callback(&state, SBP_MSG_HEARTBEAT, &swiftnav_piksi::sbpCallback<SBP_MSG_HEARTBEAT>, (void*) this, &callback_nodes_[0]);
		sbp_register_callback(&state, SBP_MSG_GPS_TIME, &swiftnav_piksi::sbpCallback<SBP_MSG_GPS_TIME>, (void*) this, &callback_nodes_[1]);
		sbp_register_callback(&state, SBP_MSG_POS_LLH, &swiftnav_piksi::sbpCallback<SBP_MSG_POS_LLH>, (void*) this, &callback_nodes_[2]);
		sbp_register_callback(&state, SBP_MSG_DOPS, &swiftnav_piksi::sbpCallback<SBP_MSG_DOPS>, (void*) this, &callback_nodes_[3]);
		sbp_register_callback(&state, SBP_MSG_BASELINE_NED, &swiftnav_piksi::sbpCallback<SBP_MSG_BASELINE_NED>, (void*) this, &callback_nodes_[4]);
		sbp_register_callback(&state, SBP_MSG_VEL_NED, &swiftnav_piksi::sbpCallback<SBP_MSG_VEL_NED>, (void*) this, &callback_nodes_[5]);

		llh_pub = nh.advertise<sensor_msgs::NavSatFix>( "gps/fix", 1 );
		rtk_pub = nh.advertise<nav_msgs::Odometry>( "gps/rtkfix", 1 );
		time_pub = nh.advertise<sensor_msgs::TimeReference>( "gps/time", 1 );


		if (publish_raw_msg_mask_ != 0) {
			raw_pub = nh.advertise<piksi_driver::Raw>( "gps/raw", 5);

			// Observation data
			sbp_register_callback(&state, SBP_MSG_OBS_DEP_B, &swiftnav_piksi::sbpCallback<SBP_MSG_OBS_DEP_B>, (void*) this, &callback_nodes_[6]);
			sbp_register_callback(&state, SBP_MSG_EPHEMERIS_DEP_C, &swiftnav_piksi::sbpCallback<SBP_MSG_EPHEMERIS_DEP_C>,this, &callback_nodes_[7]);
			sbp_register_callback(&state, SBP_MSG_BASE_POS_LLH, &swiftnav_piksi::sbpCallback<SBP_MSG_BASE_POS_LLH>, this, &callback_nodes_[8]);
			sbp_register_callback(&state, SBP_MSG_BASE_POS_ECEF, &swiftnav_piksi::sbpCallback<SBP_MSG_BASE_POS_ECEF>, this, &callback_nodes_[9]);
		}

		if (subscribe_observation_data_) {
			raw_sub = nh.subscribe("gps/raw", 5, &PiksiDriver::rawRosMsgCallback, this);
		}

		return true;
	}

	void PiksiDriver::PIKSIClose( )
	{
		boost::mutex::scoped_lock lock( cmd_lock );
		PIKSICloseNoLock( );
	}

	void PiksiDriver::PIKSICloseNoLock( )
	{
		int8_t old_piksid = piksid;
		if( piksid < 0 )
		{
			return;
		}
		piksid = -1;
		piksi_close( old_piksid );
		if( llh_pub )
			llh_pub.shutdown( );
		if( time_pub )
			time_pub.shutdown( );
	}

	void PiksiDriver::spin( )
	{
		while( ros::ok( ) )
		{
			boost::this_thread::interruption_point( );
			PiksiDriver::spinOnce( );
			heartbeat_diag.update( );
			llh_diag.update( );
			rtk_diag.update( );
			spin_rate.sleep( );
		}
	}

	void PiksiDriver::spinOnce( )
	{
		int ret;

		cmd_lock.lock( );
		if( piksid < 0 && !PIKSIOpenNoLock( ) )
		{
			cmd_lock.unlock( );
			return;
		}

		ret = sbp_process( &state, &read_data );
		cmd_lock.unlock( );
	}

	void PiksiDriver::DiagCB( diagnostic_updater::DiagnosticStatusWrapper &stat )
	{
		stat.summary( diagnostic_msgs::DiagnosticStatus::OK, "PIKSI status OK" );
		boost::mutex::scoped_lock lock( cmd_lock );

		if( piksid < 0 && !PIKSIOpenNoLock( ) )
		{
			stat.summary( diagnostic_msgs::DiagnosticStatus::ERROR, "Disconnected" );
			return;
		}
		else if( open_failure_count > last_open_failure_count )
		{
			stat.summary( diagnostic_msgs::DiagnosticStatus::ERROR, 
				"Open Failure Count Increase" );
		}
			else if( io_failure_count > last_io_failure_count )
		{
			stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, 
				"I/O Failure Count Increase" );
		}
		else if( 0 != heartbeat_flags & 0x7 )
		{
			stat.summary( diagnostic_msgs::DiagnosticStatus::ERROR, 
				"Piksi Error indicated by heartbeat flags" );
		}
		else if( num_rtk_satellites < 5 )
		{
			stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, 
				"RTK Satellite fix invalid: too few satellites in view" );
		}
		else if( rtk_status != 1 )
		{
			stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, 
				"No GPS RTK fix" );
		}

		stat.add( "io_failure_count", io_failure_count );
		last_io_failure_count = io_failure_count;

		stat.add( "open_failure_count", open_failure_count );
		last_open_failure_count = open_failure_count;

		stat.add( "Heartbeat status (0 = good)", heartbeat_flags);
		stat.add( "Number of satellites used in GPS RTK solution", num_rtk_satellites );
		stat.add( "GPS RTK solution status (1 = good)", rtk_status );
		stat.add( "GPS RTK meters north", rtk_north );
		stat.add( "GPS RTK meters east", rtk_east );
		stat.add( "GPS RTK height difference (m)", rtk_height );
		stat.add( "GPS RTK horizontal accuracy (m)", rtk_h_accuracy );
		stat.add( "GPS RTK velocity north", rtk_vel_north );
		stat.add( "GPS RTK velocity east", rtk_vel_east );
		stat.add( "GPS RTK velocity up", rtk_vel_up );
		stat.add( "Number of satellites used for lat/lon", num_llh_satellites);
		stat.add( "GPS lat/lon solution status", llh_status );
		stat.add( "GPS latitude", llh_lat );
		stat.add( "GPS longitude", llh_lon );
		stat.add( "GPS altitude", llh_height );
		stat.add( "GPS lat/lon horizontal accuracy (m)", llh_h_accuracy);
	}

	void PiksiDriver::rawRosMsgCallback(const piksi_driver::RawConstPtr& msg) {

		size_t size = msg->payload.size();
		const u8* payload = static_cast<const u8*>(&(msg->payload[0]));

		s8 res = sbp_send_message(&state, msg->msg_type, msg->sender_id, size, (u8*)payload, &send_cmd);
		if (res != SBP_OK)
			ROS_ERROR("Failed to send observation message to piksi. Code: %d", res);
	}


	void PiksiDriver::sbpCallback(u16 msg_type, u16 sender_id, u8 len, u8 msg[])
	{
		// Publish raw?
		if ((msg_type & publish_raw_msg_mask_) != 0) {
			piksi_driver::RawPtr raw(new piksi_driver::Raw());

			raw->sender_id = sender_id;
			raw->msg_type = msg_type;
			std::vector<u8> payload_vector(msg, msg + len);
			raw->payload.swap(payload_vector);

			raw_pub.publish(raw);
		}

		switch (msg_type) {
		case SBP_MSG_HEARTBEAT:
		{
			msg_heartbeat_t hb = *(msg_heartbeat_t*) msg;
			heartbeat_pub_freq.tick();
			heartbeat_flags |= (hb.flags & 0x7);    // accumulate errors for diags
			break;
		}
		case SBP_MSG_GPS_TIME:
		{
			msg_gps_time_t time = *(msg_gps_time_t*) msg;

			sensor_msgs::TimeReferencePtr time_msg( new sensor_msgs::TimeReference );

			time_msg->header.frame_id = frame_id;
			time_msg->header.stamp = ros::Time::now( );

			time_msg->time_ref.sec = time.tow;
			time_msg->source = "gps";

			time_pub.publish( time_msg );
			break;
		}
		case SBP_MSG_POS_LLH:
		{
			msg_pos_llh_t llh = *(msg_pos_llh_t*) msg;

			sensor_msgs::NavSatFixPtr llh_msg( new sensor_msgs::NavSatFix );

			llh_msg->header.frame_id = frame_id;
			llh_msg->header.stamp = ros::Time::now( );

			llh_msg->status.status = 0;
			llh_msg->status.service = 1;

			llh_msg->latitude = llh.lat;
			llh_msg->longitude = llh.lon;
			llh_msg->altitude = llh.height;

			// populate the covariance matrix
			// FIXME: llh.h/v_accuracy doesn't work yet, so use HDOP temporarily
			// knowing that it's wrong, but in the ballpark
			//double h_covariance = llh.h_accuracy * llh.h_accuracy;
			//double v_covariance = llh.v_accuracy * llh.v_accuracy;
			double h_covariance = hdop * hdop;
			double v_covariance = hdop * hdop;
			llh_msg->position_covariance[0]  = h_covariance;   // x = 0, 0
			llh_msg->position_covariance[4]  = h_covariance;   // y = 1, 1
			llh_msg->position_covariance[8]  = v_covariance;   // z = 2, 2

			llh_pub.publish( llh_msg );

			// populate diagnostic data
			llh_pub_freq.tick( );
			llh_status |= llh.flags;
			num_llh_satellites = llh.n_sats;
			llh_lat = llh.lat;
			llh_lon = llh.lon;
			llh_height = llh.height;
			// FIXME: llh_h_accuracy doesn't work yet, so use hdop
			//driver->llh_h_accuracy = llh.h_accuracy / 1000.0;
			llh_h_accuracy = hdop;

			break;
		}
		case SBP_MSG_DOPS:
		{
			msg_dops_t dops = *(msg_dops_t*) msg;

			// FIXME: this is incorrect, but h_accuracy doesn't work yet
			llh_h_accuracy = dops.hdop;
			//driver->heartbeat_pub_freq.tick();
			break;
		}
		case SBP_MSG_BASELINE_NED:
		{
			msg_baseline_ned_t sbp_ned = *(msg_baseline_ned_t*) msg;

			nav_msgs::OdometryPtr rtk_odom_msg( new nav_msgs::Odometry );

			rtk_odom_msg->header.frame_id = frame_id;
			// For best accuracy, header.stamp should maybe get tow converted to ros::Time
			rtk_odom_msg->header.stamp = ros::Time::now( );

			// convert to meters from mm, and NED to ENU
			rtk_odom_msg->pose.pose.position.x = sbp_ned.e/1000.0;
			rtk_odom_msg->pose.pose.position.y = sbp_ned.n/1000.0;
			rtk_odom_msg->pose.pose.position.z = -sbp_ned.d/1000.0;

			// Set orientation to 0; GPS doesn't provide orientation
			rtk_odom_msg->pose.pose.orientation.x = 0;
			rtk_odom_msg->pose.pose.orientation.y = 0;
			rtk_odom_msg->pose.pose.orientation.z = 0;
			rtk_odom_msg->pose.pose.orientation.w = 0;

			float h_covariance = 1.0e3;
			float v_covariance = 0.0e3;

			// populate the pose covariance matrix if we have a good fix
			if ( 1 == sbp_ned.flags && 4 < sbp_ned.n_sats)
			{
				// FIXME: h_accuracy doesn't work yet, so use hard-coded 4cm
				// until it does
				//h_covariance = (sbp_ned.h_accuracy * sbp_ned.h_accuracy) / 1.0e-6;
				//v_covariance = (sbp_ned.v_accuracy * sbp_ned.v_accuracy) / 1.0e-6;
				h_covariance = rtk_h_accuracy * rtk_h_accuracy;
				v_covariance = rtk_h_accuracy * rtk_h_accuracy;
			}

			// Pose x/y/z covariance is whatever we decided h & v covariance is
			rtk_odom_msg->pose.covariance[0]  = h_covariance;   // x = 0, 0 in the 6x6 cov matrix
			rtk_odom_msg->pose.covariance[7]  = h_covariance;   // y = 1, 1
			rtk_odom_msg->pose.covariance[14] = v_covariance;  // z = 2, 2

			// default angular pose to unknown
			rtk_odom_msg->pose.covariance[21] = 1.0e3;  // x rotation = 3, 3
			rtk_odom_msg->pose.covariance[28] = 1.0e3;  // y rotation = 4, 4
			rtk_odom_msg->pose.covariance[35] = 1.0e3;  // z rotation = 5, 5

			// Populate linear part of Twist with last velocity reported: by vel_ned_callback
			rtk_odom_msg->twist.twist.linear.x = rtk_vel_east;
			rtk_odom_msg->twist.twist.linear.y = rtk_vel_north;
			rtk_odom_msg->twist.twist.linear.z = rtk_vel_up;

			// Set angular velocity to 0 - GPS doesn't provide angular velocity
			rtk_odom_msg->twist.twist.angular.x = 0;
			rtk_odom_msg->twist.twist.angular.y = 0;
			rtk_odom_msg->twist.twist.angular.z = 0;

			// set up the Twist covariance matrix
			// FIXME: I don't know what the covariance of linear velocity should be.
			// 12/19 asked on swiftnav google group
			// GPS doesn't provide rotationl velocity
			rtk_odom_msg->twist.covariance[0]  = 1.0e3;   // x velocity = 0, 0 in the 6x6 cov matrix
			rtk_odom_msg->twist.covariance[7]  = 1.0e3;   // y velocity = 1, 1
			rtk_odom_msg->twist.covariance[14] = 1.0e3;  // z velocity = 2, 2
			rtk_odom_msg->twist.covariance[21] = 1.0e3;  // x rotational velocity = 3, 3
			rtk_odom_msg->twist.covariance[28] = 1.0e3;  // y rotational velocity = 4, 4
			rtk_odom_msg->twist.covariance[35] = 1.0e3;  // z rotational velocity = 5, 5

			rtk_pub.publish( rtk_odom_msg );

			// save diagnostic data
			rtk_pub_freq.tick( );
			rtk_status = sbp_ned.flags;
			num_rtk_satellites = sbp_ned.n_sats;
			rtk_north = rtk_odom_msg->pose.pose.position.x;
			rtk_east = rtk_odom_msg->pose.pose.position.y;
			rtk_height = rtk_odom_msg->pose.pose.position.z;
			// FIXME: rtk.h_accuracy doesn't work yet
			//rtk_h_accuracy = rtk.h_accuracy / 1000.0;

			break;
		}
		case SBP_MSG_VEL_NED:
		{
			msg_vel_ned_t sbp_vel = *(msg_vel_ned_t*) msg;

			// save velocity in the Twist member of a private Odometry msg, from where it
			// will be pulled to populate a published Odometry msg next time a
			// msg_baseline_ned_t message is received
			rtk_vel_north = sbp_vel.n/1000.0;
			rtk_vel_east = sbp_vel.e/1000.0;
			rtk_vel_up = -sbp_vel.d/1000.0;
			break;
		}
		}
		return;
	}
}
