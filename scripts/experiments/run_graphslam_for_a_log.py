
import os
import sys
import argparse
from experiments import *


def run_odom_calib(carmen_path, log_path, output_dir, config_dir):
    global GPS_TO_USE, PARAM_FILE
    program = carmen_path + "/src/odometry_calibration/calibrate_bias_from_log"
    output_path = output_dir + "/odom_calib.txt"
    report_path = output_dir + "/report_odom_calib.txt"
    poses_opt_path = output_dir + "/poses-opt_odom_calib.txt"
    additional_args = " -n 200 -i 50 --view 0 --max_multiplicative_v 1.3 --min_multiplicative_v 0.7 --max_multiplicative_phi 1.3 --min_multiplicative_phi 0.7 --max_additive_phi 0.5 --min_additive_phi -0.5 --gps_to_use %d " % GPS_TO_USE
    config_path = ""
    cmd = "%s %s %s %s %s %s %s --save %s" % (program, log_path, PARAM_FILE, output_path,
                                              report_path, poses_opt_path, additional_args, config_dir + "/odom_calib_config.txt")
    run_command(cmd)

    # visualization
    program = "python scripts/visualization_and_analysis/generate_odometry_calibration_image.py"
    img_path = output_dir + "/image_odom_calib.png"
    cmd = "%s %s %s" % (program, report_path, img_path)
    run_command(cmd)


# mode = [fused_odom | graphslam]
def run_graphslam(carmen_path, log_path, output_dir, mode, config_dir):
    global GPS_XY_STD, GPS_H_STD, LOOPS_XY_STD, LOOPS_H_STD, GPS_TO_USE, INTENSITY_MODE, PARAM_FILE

    program = "graphslam_fast/graphslam"

    if mode == "fused_odom":
        output_path = output_dir + "/fused_odom.txt"
        img_path = output_dir + "/image_fused_odom.png"
        loops = ""
    elif mode == "graphslam":
        output_path = output_dir + "/graphslam.txt"
        img_path = output_dir + "/image_graphslam.png"
        loops = "-l %s/loops.txt --pf_loops %s/localization_loops.txt" % (
            output_dir, output_dir)
    else:
        raise Exception("Invalid mode '%s'" % mode)

    if ("brt" in log_path):
        args = "--gps_xy_std 20.00000 --gps_angle_std 20.000000 --pf_loops_xy_std 0.10000 --pf_loops_angle_std 3.000000 --gps_step 50"
    elif ("jardim" in log_path):
        args = "--gps_xy_std 5.00000 --gps_angle_std 20.000000 --pf_loops_xy_std 0.05 --pf_loops_angle_std 1.0 "
    else:
        args = " --gps_xy_std 2.000000 --gps_angle_std 20.000000 --pf_loops_xy_std 0.05 --pf_loops_angle_std 1.0 "

    args += " --gps_id %d" % GPS_TO_USE
    args += " -i " + INTENSITY_MODE

    odom_calib = output_dir + "/odom_calib.txt"
    cmd = "%s %s %s %s -o %s %s %s --save %s" % (program, log_path, PARAM_FILE,
                                                 output_path, odom_calib, loops, args, config_dir + "/" + mode + "_config.txt")
    run_command(cmd)

    # visualization
    program = "python scripts/visualization_and_analysis/generate_graphslam_image.py"
    cmd = "%s %s %s" % (program, output_path, img_path)
    run_command(cmd)


def run_loop_closures(carmen_path, log_path, output_dir, mode, config_dir):
    global IGNORE_POINTS_ABOVE, IGNORE_POINTS_BELOW, SKIP_WHEN_VELOCITY_IS_BELOW, GPS_TO_USE, INTENSITY_MODE, PARAM_FILE

    program = "./gicp/generate_loop_closures"
    odom_calib = output_dir + "/odom_calib.txt"
    fused_odom = output_dir + "/fused_odom.txt"

    cmd = " %s %s %s -o %s -f %s --gps_id %d -i %s --start_paused 0 --save %s" % (
        program, log_path, PARAM_FILE, odom_calib, fused_odom, GPS_TO_USE, INTENSITY_MODE, config_dir + "/loops_" + mode + "_config.txt")

    if mode == "gicp":
        gicp_args = " --mode gicp --dist_to_accumulate 2.0 --ignore_above_threshold %lf --ignore_below_threshold %lf --v_thresh %lf --clean_map 1 --view_imgs 0 --view_pointcloud 0" % (
            IGNORE_POINTS_ABOVE, IGNORE_POINTS_BELOW, SKIP_WHEN_VELOCITY_IS_BELOW)
        gicp_output = " " + output_dir + "/loops.txt"
        run_command(cmd + gicp_output + gicp_args)

    elif mode == "particle_filter":
        pf_args = " --mode particle_filter --n_particles 200 --gps_xy_std 5.0 --gps_h_std 20 --dist_to_accumulate 20.0 --loop_dist 5.0 --n_corrections_when_reinit 20 --v_thresh %lf --clean_map 1 --view_imgs 0 --view_pointcloud 0" % (
            SKIP_WHEN_VELOCITY_IS_BELOW)
        pf_output = " " + output_dir + "/pf_loops.txt"
        run_command(cmd + pf_output + pf_args)
    elif mode == "localization":
        loop_closure_time = 10

        # if ('aeroport' in log_path) or ('estac' in log_path) or ('ambient' in log_path) or ('honof' in log_path):
        # loop_closure_time = 10

        if 'aeroport' in log_path:
            camera_latency = 0.43
        elif 'noite' in log_path:
            camera_latency = 0.3
        else:
            camera_latency = 0.0

        loc_args = " --mode localization --n_particles 200 --gps_xy_std 3.0 --gps_h_std 20 --dist_to_accumulate 20.0 --loop_dist 10.0 --n_corrections_when_reinit 20 --v_thresh %lf -v 1 --time_dist %lf --v_std 1.0 --phi_std 1.0 --odom_xy_std 0.1 --odom_h_std 1.0 --color_red_std 3 --color_green_std 3 --color_blue_std 3 --reflectivity_std 3 --use_map_weight 1 --clean_map 1 --view_imgs 0 --view_pointcloud 0 --tile_size 70 --camera_latency %lf" % (
            SKIP_WHEN_VELOCITY_IS_BELOW, loop_closure_time, camera_latency)
        loc_output = " " + output_dir + "/localization_loops.txt"
        run_command(cmd + loc_output + loc_args + " > /dev/null 2>&1")
    else:
        sys.exit(print("Error: Invalid mode '%s'" % mode))


def main(log_path, skip_until):
    global DATA_DIR

    if not os.path.exists(DATA_DIR):
        os.mkdir(DATA_DIR)

    carmen_path = os.getenv("CARMEN_HOME")
    output_dir = create_output_dir(log_path)
    config_dir = output_dir + '/configs/'

    if not os.path.exists(config_dir):
        os.mkdir(config_dir)

#    if (not skip_until) or (skip_until <= 1):
#        run_odom_calib(carmen_path, log_path, output_dir, config_dir)
#     if (not skip_until) or (skip_until <= 2):
#         run_graphslam(carmen_path, log_path, output_dir,
#                       "fused_odom", config_dir)
# if (not skip_until) or (skip_until <= 3):
# run_loop_closures(carmen_path, log_path, output_dir, 'gicp')
# if (not skip_until) or (skip_until <= 4):
# run_loop_closures(carmen_path, log_path, output_dir, 'particle_filter')
#    if (not skip_until) or (skip_until <= 5):
#        run_loop_closures(carmen_path, log_path, output_dir,
#                          'localization', config_dir)
    if (not skip_until) or (skip_until <= 6):
        run_graphslam(carmen_path, log_path, output_dir,
                      "graphslam", config_dir)
    if skip_until and skip_until > 6:
        print("Skipped all steps.")

    print("Done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Script to optimize poses of a log.')
    parser.add_argument('log', help='Path to a log.')
    parser.add_argument(
        '--skip', help='Skip all steps of optimization until the chosen one. Possible values: [1. odom_calib | 2. fused_odom | 3. loops with gicp | 4. loops with pf | 5. loops with localization | 6. graphslam]', type=int)
    args = parser.parse_args()
    main(args.log, args.skip)
