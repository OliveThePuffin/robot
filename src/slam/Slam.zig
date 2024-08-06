const std = @import("std");
const log = @import("logger").log;
const rs_depth = @import("rs-depth.zig");
const KalmanFilter = @import("KalmanFilter.zig").KalmanFilter(2, 1, 1);
const Slam = @This();

const name = "SLAM";
var gpa = std.heap.GeneralPurposeAllocator(.{}){};
var allocator = gpa.allocator();
var kf: KalmanFilter = undefined;

pub const Config = struct {
    dry_run: bool,
    kalman_filter: KalmanFilter.Config,
};

pub fn start() void {
    log(.INFO, name, "Starting", .{});
    rs_depth.start_loop();
}

pub fn stop() void {
    log(.INFO, name, "Stopping", .{});
    rs_depth.stop_loop();
}

pub fn init(config: Config) !void {
    log(.INFO, name, "Initializing", .{});

    kf = try KalmanFilter.init(
        config.kalman_filter,
        [1][2]f32{.{ 1, 0 }}, // H
        [1][1]f32{.{400}}, // R
        null, // x
        [2][2]f32{ .{ 1, 0.25 }, .{ 0, 1 } }, // F
        [2][1]f32{ .{0.0313}, .{0.25} }, // G
        [2][2]f32{ .{ 500, 0 }, .{ 0, 500 } }, // P
        [2][2]f32{ .{ 9.765625e-6, 7.8125e-3 }, .{ 7.8125e-3, 6.25e-2 } }, // Q
    );

    const x = try kf.iterate(
        [1]f32{6.43}, // z
        null, // H
        null, // R
        [1]f32{39.81 - 9.81}, // u
        null, // F
        null, // G
        null, // Q
    );

    log(.INFO, name, "x_1: {d}", .{x});

    rs_depth.module_config = .{ .dry_run = config.dry_run };
}

pub fn deinit() void {
    log(.INFO, name, "Deinitializing", .{});

    if (gpa.deinit() == .leak) {
        log(.ERROR, name, "Memory leaks detected", .{});
    }
}

// TODO: also try adding a kalman filter here

test {}
