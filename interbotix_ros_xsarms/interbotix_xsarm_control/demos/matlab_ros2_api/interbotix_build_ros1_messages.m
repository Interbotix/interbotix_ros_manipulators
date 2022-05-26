function success = interbotix_build_ros1_messages(options)
    % Run this function to build the Interbotix ROS Messages for use with MATLAB
    arguments
        options.py27path string = "/usr/bin/python2.7" % absolute path to python2.7 executable
    end

    success = false;

    rel_core_msgs_path = "../../../../../interbotix_ros_core/interbotix_ros_xseries/matlab_msg_gen_ros1/";

    try % try setting the Python version required to build ros messages
        pyenv("Version", options.py27path);
    catch ME
        warning("Try setting your pyenv to Version 2.7 when you first open MATLAB.")
        rethrow(ME)
    end
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
