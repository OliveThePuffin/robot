const std = @import("std");
const Log = @import("Log").Log;
const LogConfig = @import("Log").Config;
const RSDepth = @import("rs-depth.zig").RealsenseDepth;
const RSConfig = @import("rs-depth.zig").Config;
const KalmanFilter = @import("KalmanFilter.zig").KalmanFilter(2, 1, 1);
const IKDTree = @import("IKDTree.zig").IKDTree;
const Slam = @This();

const I3DTree = IKDTree(3);
var log: Log = undefined;
var gpa = std.heap.GeneralPurposeAllocator(.{}){};
var allocator = gpa.allocator();
var kf: KalmanFilter = undefined;
var rs_depth: RSDepth = undefined;

pub const Config = struct {
    log_config: LogConfig,
    kalman_filter: KalmanFilter.Config,
    ikd_tree: I3DTree.Config,
    rs_config: RSConfig,
};

pub fn start() void {
    // Start threads:
    // Get IMU input (100-250Hz)
    // Get Lidar input (100k-500kHz)
    log.info("Starting", .{});
    rs_depth.start_loop();
}

pub fn stop() void {
    log.info("Stopping", .{});
    rs_depth.stop_loop();
}

pub fn init(config: Config) !void {
    log = try Log.init(config.log_config);
    log.info("Initializing Slam", .{});

    rs_depth = try RSDepth.init(config.rs_config);
}

pub fn deinit() void {
    rs_depth.deinit();

    log.info("Deinitializing", .{});
    log.deinit();

    if (gpa.deinit() == .leak) {
        log.err("Memory leaks detected", .{});
    }
}

test {}
