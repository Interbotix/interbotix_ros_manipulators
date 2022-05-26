function success = interbotix_matlab_ros_setup()
    % Run this file before using the xs arm examples

    success = false;

    here = pwd;
    rel_core_msgs_path = "/../../../../../interbotix_ros_core/interbotix_ros_xseries/matlab_msg_gen_ros1/";
    rel_mr_lib         = "/../../../../../interbotix_ros_toolboxes/third_party_libraries/ModernRobotics/mr";

    fprintf("Adding interbotix_xs_msgs to path...")
    if (isfolder(rel_core_msgs_path + "glnxa64/install/m"))
        addpath(rel_core_msgs_path + "glnxa64/install/m")
        if any(strcmp(rosmsg("list"), "interbotix_xs_msgs/JointSingleCommand"))
            fprintf(" Done\n")
        else
            fprintf("\n")
            warning("Interbotix messages directory was found but messages were not added to the path. Try deleting the 'matlab_msg_gen_ros1' directory and rebuilding using the interbotix_build_ros1_messages function.")
            return
        end
    else
        fprintf("\n")
        warning("Could not find interbotix messages. Were they built using the interbotix_build_ros1_messages function?")
        return
    end

    % adds modern robotics library to path
    fprintf("Adding Modern Robotics library to path... \b")
    if (isfolder(rel_mr_lib))
        addpath(rel_mr_lib)
        try
            NearZero(0.0);
            fprintf(" Done\n")
        catch
            fprintf("\n")
            fprintf("Modern Robotics library's directory was added to the path but basic functions were not found.\n")
            return
        end
    else
        fprintf("\n")
        warning("Could not find the Modern Robotics library. Was the ModernRobotics submodule updated in 'interbotix_ros_toolboxes/third_party_libraries'?")
        return
    end
    addpath(rel_mr_lib)

    fprintf("Done with startup. You can now run MATLAB scripts using the xs modules.\n")

    cd(here)
    fprintf("Running in directory '%s'\n", here)

    success = true;
end
