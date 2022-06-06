function success = interbotix_build_ros1_messages()
    % Run this function to build the Interbotix X-Series ROS Messages for use with MATLAB
    % Some users have faced issues with MATLAB release 2022a that were fixed following the advice here:
    %   https://www.mathworks.com/matlabcentral/answers/1690800-error-in-building-custom-ros-messages

    success = false;

    rel_core_msgs_path = "../../../../interbotix_ros_core/interbotix_ros_xseries/matlab_msg_gen_ros1/";

    if ~exist(rel_core_msgs_path, 'dir') && ~any(strcmp(rosmsg("list"), "interbotix_xs_msgs/JointSingleCommand"))
        disp("Building messages...")
        rosgenmsg(rel_core_msgs_path + "../")
        addpath(rel_core_msgs_path + "glnxa64/install/m")
        savepath
        clear classes
        rehash toolboxcache
        if any(strcmp(rosmsg("list"), "interbotix_xs_msgs/JointSingleCommand"))
            disp("Done building messages. You can now run MATLAB scripts using the messages.")
            disp("Make sure to restart MATLAB before running any scripts.")
            success = true;
        else
            warning("Something went wrong when building messages.")
            success = false;
        end
    else
        disp("Messages already built. Skipping...")
        success = true;
    end
end
