/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <tf2_ros/static_transform_broadcaster.h>
#include <Eigen/Geometry>

#include <boost/program_options.hpp>
namespace po=boost::program_options;

static void usage (const char* prog_name, const po::options_description &opts, bool desc=false) {
  if (desc) {
    std::cout << "A command line utility for manually defining a (static) transform" << std::endl;
    std::cout << "from parent_frame_id to child_frame_id." << std::endl;
  }
  std::cout << std::endl;
  std::cout << "Usage: static_transform_publisher [options] x y z  <rotation> parent_frame_id child_frame_id" << std::endl;
  std::cout << opts << std::endl;
}

static void parse_arguments(int argc, char **argv,
                            geometry_msgs::TransformStamped &msg) {
  std::string mode;
  po::options_description options_description("allowed options");
  options_description.add_options()
      ("help,h", "show this help message")
      ("mode,m", po::value<std::string>(&mode))
      ;

  po::variables_map variables_map;
  std::vector<std::string> args;
  std::vector<std::string>::const_iterator arg;
  try {
    po::parsed_options parsed =
        po::command_line_parser(argc, argv)
        .options(options_description)
        .allow_unregistered()
        .run();

    po::store(parsed, variables_map);
    po::notify(variables_map);
    args = po::collect_unrecognized(parsed.options, po::include_positional);
    arg = args.begin();

    if (variables_map.count("help")) {
      usage(argv[0], options_description, true);
      exit (EXIT_SUCCESS);
    }
    const size_t numArgs = 3 + 2;

    bool bQuatMode = (mode == "wxyz" || mode == "xyzw");
    if (mode == "")
    {
      if (args.size() == numArgs+4) {
        bQuatMode = true; // 4 rotational args trigger quaternion mode too
        mode = "xyzw";
      } else if (args.size() == numArgs+3) {
        mode = "zyx";
      } else {
        throw po::error("invalid number of positional arguments");
      }
    }

    // consume position arguments
    msg.transform.translation.x = boost::lexical_cast<double>(*arg); ++arg;
    msg.transform.translation.y = boost::lexical_cast<double>(*arg); ++arg;
    msg.transform.translation.z = boost::lexical_cast<double>(*arg); ++arg;

    // consume orientation arguments
    Eigen::Quaterniond q;
    if (bQuatMode) { // parse Quaternion
      if (args.size() != numArgs+4)
        throw po::error("quaternion mode requires " +
                        boost::lexical_cast<std::string>(numArgs+4) +
                        " positional arguments");

      const std::string eigen_order("xyzw");
      double data[4];
      for (size_t i=0; i<4; ++i) {
        size_t idx = eigen_order.find(mode[i]);
        data[idx] = boost::lexical_cast<double>(*arg); ++arg;
      }
      q = Eigen::Quaterniond(data);

    } else { // parse Euler angles
      if (args.size() != numArgs+3)
        throw po::error("Euler angles require " +
                        boost::lexical_cast<std::string>(numArgs+3) +
                        " positional arguments");
      if (mode.size() != 3)
        throw po::error("mode specification for Euler angles requires a string from 3 chars (xyz)");

      const std::string axes_order("xyz");
      size_t axes_idxs[3];
      double angles[3];

      for (size_t i=0; i<3; ++i) {
        size_t idx = axes_order.find(mode[i]);
        if (idx == std::string::npos)
          throw po::error("invalid axis specification for Euler angles: " +
                          boost::lexical_cast<std::string>(mode[i]));
        axes_idxs[i] = idx;
        angles[i] = boost::lexical_cast<double>(*arg); ++arg;
      }
      q = Eigen::AngleAxisd(angles[0], Eigen::Vector3d::Unit(axes_idxs[0])) *
          Eigen::AngleAxisd(angles[1], Eigen::Vector3d::Unit(axes_idxs[1])) *
          Eigen::AngleAxisd(angles[2], Eigen::Vector3d::Unit(axes_idxs[2]));
    }
    // assign quaternion
    q.normalize();
    msg.transform.rotation.x = q.x();
    msg.transform.rotation.y = q.y();
    msg.transform.rotation.z = q.z();
    msg.transform.rotation.w = q.w();

    // consume link arguments
    msg.header.frame_id = *arg++;
    msg.child_frame_id = *arg++;
  } catch (const po::error  &e) {
    ROS_FATAL_STREAM(e.what());
    usage(argv[0], options_description);
    exit (EXIT_FAILURE);
  } catch (const boost::bad_lexical_cast &e) {
    ROS_FATAL_STREAM("failed to parse numerical value: " << *arg);
    usage(argv[0], options_description);
    exit (EXIT_FAILURE);
  }
}

int main(int argc, char ** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "static_transform_publisher", ros::init_options::AnonymousName);

  geometry_msgs::TransformStamped msg;
  parse_arguments(argc, argv, msg);

  if (msg.header.frame_id.empty() || msg.child_frame_id.empty())
  {
    ROS_FATAL("target or source frame is empty");
    exit(1);
  }
  if (msg.header.frame_id == msg.child_frame_id)
  {
    ROS_FATAL("target and source frame are the same (%s, %s) this cannot work",
              msg.child_frame_id.c_str(), msg.header.frame_id.c_str());
    exit(1);
  }

  tf2_ros::StaticTransformBroadcaster broadcaster;
  broadcaster.sendTransform(msg);

  ROS_INFO("Spinning until killed, publishing %s to %s",
           msg.header.frame_id.c_str(), msg.child_frame_id.c_str());
  ros::spin();

  return 0;
}
