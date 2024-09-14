const Dir = @import("std").fs.Dir;
const Slam = @import("Slam");
const LogConfig = @import("logger").Config;
const Config = @This();

slam: Slam.Config,

const opencl_gpu = .{
    .platform_name = "NVIDIA CUDA",
    .device_name = "NVIDIA GeForce RTX 3080",
};

const default_ikd_tree = .{
    .a_bal = 0.75,
    .a_del = 0.5,
    .relative_tolrance = 0.00001,
};

const default_slam = .{
    .log_config = .{
        .level = .INFO,
        .abs_path = null,
        .channel = "Slam",
    },
    .kalman_filter = .{
        .opencl = opencl_gpu,
    },
    .ikd_tree = default_ikd_tree,
    .rs_config = .{
        .dry_run = false,
        .log_config = .{
            .level = .INFO,
            .abs_path = "/tmp/CLAW",
            .channel = "Realsense",
        },
    },
};

pub const default = Config{
    .slam = default_slam,
};

pub const dry_run = Config{
    .slam = .{
        .log_config = .{
            .level = .DEBUG,
            .abs_path = null,
            .channel = "Slam",
        },
        .kalman_filter = .{
            .opencl = opencl_gpu,
        },
        .ikd_tree = default_ikd_tree,
        .rs_config = .{
            .dry_run = true,
            .log_config = .{
                .level = .DEBUG,
                .abs_path = null,
                .channel = "Realsense",
            },
        },
    },
};
