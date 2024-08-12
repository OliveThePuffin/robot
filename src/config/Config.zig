const Slam = @import("Slam");
const LogLevel = @import("logger").Level;
const Config = @This();

log_level: LogLevel,
slam: Slam.Config,

pub const default = Config{
    .log_level = .INFO,
    .slam = .{
        .dry_run = false,
        .kalman_filter = .{
            .opencl = .{
                .platform_name = "NVIDIA CUDA",
                .device_name = "NVIDIA GeForce RTX 3080",
            },
        },
    },
};

pub const dry_run = Config{
    .log_level = .DEBUG,
    .slam = .{
        .dry_run = true,
        .kalman_filter = .{
            .opencl = .{
                .platform_name = "NVIDIA CUDA",
                .device_name = "NVIDIA GeForce RTX 3080",
            },
        },
    },
};

pub const cpu_dry_run = Config{
    .log_level = .DEBUG,
    .slam = .{
        .dry_run = true,
        .kalman_filter = .{
            .opencl = .{
                .platform_name = "Clover",
                .device_name = "AMD Radeon Graphics (radeonsi, raphael_mendocino, LLVM 18.1.8, DRM 3.54, 6.6.44_1)",
            },
        },
    },
};
