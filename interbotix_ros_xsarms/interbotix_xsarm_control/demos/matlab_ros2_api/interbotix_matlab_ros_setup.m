function success = interbotix_matlab_ros_setup(options)
    % Run this file before using the xs arm examples    
    arguments
        options.py3path string = "/usr/bin/python3.8"
    end

    success = false;

    here = pwd;
    rel_core_msgs_path      = "../../../../interbotix_ros_core/interbotix_ros_xseries/matlab_msg_gen_ros1/";
    rel_common_modules_path = "../../../../interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_common_modules/src/interbotix_common_modules";

    disp("Adding interbotix_xs_msgs to path...")
    addpath(rel_core_msgs_path + "glnxa64/install/m")

    % adds interbotix matlab modules to path
    disp("Adding XS modules to path...")
    addpath("../../../../interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_modules/src/interbotix_xs_modules")

    % adds modern robotics library to path
    disp("Adding Modern Robotics library to path...")
    addpath("../../../../interbotix_ros_toolboxes/third_party_libraries/ModernRobotics/mr")

    disp("Adding Python to path...")
    if pyenv().Status == "Loaded"
        if str2double(pyenv().Version) < 3.0
            warning("Python version should be >3.6! You may have trouble running some functions!")
            warning("Restart MATLAB and try to run pyenv with a supported version.")
            return
        end
    else
        pyenv("Version", options.py3path);
    end

    disp("Adding interbotix_common_modules to path...")
    pysyspath = py.sys.path;
    pysyspath.append(rel_common_modules_path);
    try
        % try to import a module from common modules to test if imports work properly
        ang = py.importlib.import_module('angle_manipulation');
        clear ang;
    catch ME
        warning("Could not import interbotix_common_modules.angle_manipulation library. Check Python Path.")
        rethrow(ME)
    end

    disp("Done with startup. You can now run MATLAB scripts using the xs modules.")
    
    cd(here)
    fprintf("Running in directory '%s'\n", here)

    success = true;

end
