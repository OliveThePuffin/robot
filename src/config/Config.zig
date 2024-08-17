const Slam = @import("Slam");
const LogLevel = @import("logger").Level;
const Config = @This();

log_level: LogLevel,
slam: Slam.Config,

const opencl_gpu = .{
    .platform_name = "NVIDIA CUDA",
    .device_name = "NVIDIA GeForce RTX 3080",
};

const opencl_cpu = .{
    .platform_name = "Clover",
    .device_name = "AMD Radeon Graphics (radeonsi, raphael_mendocino, LLVM 18.1.8, DRM 3.54, 6.6.44_1)",
};

const default_ikd_tree = .{
    .a_bal = 0.75,
    .a_del = 0.5,
    .enable_parallel = false,
    .max_size_single_thread_rebuild = 1,
};

const default_slam = .{
    .dry_run = false,
    .ikd_tree = default_ikd_tree,
    .kalman_filter = .{
        .opencl = opencl_gpu,
    },
};

pub const default = Config{
    .log_level = .INFO,
    .slam = default_slam,
};

pub const dry_run = Config{
    .log_level = .DEBUG,
    .slam = .{
        .dry_run = true,
        .ikd_tree = default_ikd_tree,
        .kalman_filter = .{
            .opencl = opencl_gpu,
        },
    },
};

pub const cpu_dry_run = Config{
    .log_level = .DEBUG,
    .slam = .{
        .dry_run = true,
        .ikd_tree = default_ikd_tree,
        .kalman_filter = .{
            .opencl = opencl_cpu,
        },
    },
};
