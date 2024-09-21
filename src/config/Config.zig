const Dir = @import("std").fs.Dir;
const Slam = @import("Slam");
const LogConfig = @import("logger").Config;
const Config = @This();

slam: Slam.FastLIO.Config,

// DIRS
const rundir = "/tmp/CLAW/run";
const perfdir = "/tmp/CLAW/perf";
const testdir = "/tmp/CLAW/test";
const slam_rundir = rundir ++ "/Slam";

// LOGGING
const log_level = .DEBUG;

const slam_log = .{
    .level = log_level,
    .abs_path = slam_rundir,
    .channel = "Slam",
};
const realsense_log = .{
    .level = log_level,
    .abs_path = slam_rundir,
    .channel = "Realsense",
};

// OPENCL
const opencl_gpu = .{
    .platform_name = "NVIDIA CUDA",
    .device_name = "NVIDIA GeForce RTX 3080",
};

// IKD TREE
const default_ikd_tree = .{
    .a_bal = 0.75,
    .a_del = 0.5,
    .relative_tolrance = 0.00001,
};

// SLAM
const slam_freq = 2;
const default_slam = .{
    .log = slam_log,
    .realsense = .{
        .log = realsense_log,
        .dry_run = false,
    },
    .kalman_filter = .{
        .opencl = opencl_gpu,
    },
    .ikd_tree = default_ikd_tree,
    .frequency = slam_freq,
};

pub const default = Config{
    .slam = default_slam,
};

pub const dry_run = Config{
    .slam = .{
        .log = slam_log,
        .realsense = .{
            .log = realsense_log,
            .dry_run = true,
        },
        .kalman_filter = .{
            .opencl = opencl_gpu,
        },
        .ikd_tree = default_ikd_tree,
        .frequency = slam_freq,
    },
};
