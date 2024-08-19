const std = @import("std");
const log = @import("logger").log;
const rs_depth = @import("rs-depth.zig");
const KalmanFilter = @import("KalmanFilter.zig").KalmanFilter(2, 1, 1);
const IKDTree = @import("IKDTree.zig").IKDTree;
const Slam = @This();

const name = "SLAM";
const I3DTree = IKDTree(3);
var gpa = std.heap.GeneralPurposeAllocator(.{}){};
var allocator = gpa.allocator();
var kf: KalmanFilter = undefined;

pub const Config = struct {
    dry_run: bool,
    kalman_filter: KalmanFilter.Config,
    ikd_tree: I3DTree.Config,
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

    var points = try allocator.alloc([3]f32, 100 * 100 * 100);
    for (0..100) |i| {
        for (0..100) |j| {
            for (0..100) |k| {
                points[i * 100 * 100 + j * 100 + k] = .{
                    @as(f32, @floatFromInt(i)) / 20,
                    @as(f32, @floatFromInt(j)) / 20,
                    @as(f32, @floatFromInt(k)) / 20,
                };
            }
        }
    }

    // Making a fixed buffer allocator makes the ikd tree much faster
    const buffer: []u8 = try gpa.allocator().alloc(u8, 1024 * 1024 * 1024);
    var fba = std.heap.FixedBufferAllocator.init(buffer);
    defer allocator.free(buffer);

    var ikd = try I3DTree.init(points, config.ikd_tree, fba.allocator());
    defer allocator.free(points);

    const closest_ikd = ikd.nearest(.{ 0.5, 0.5, 0.5 }).?;
    log(.INFO, name, "Closest IKD: {}, {}, {}", .{ closest_ikd[0], closest_ikd[1], closest_ikd[2] });

    var closest_arr: [3]f32 = .{ 0, 0, 0 };
    for (points) |p| {
        const d1 = (p[0] - closest_arr[0]) * (p[0] - closest_arr[0]) + (p[1] - closest_arr[1]) * (p[1] - closest_arr[1]) + (p[2] - closest_arr[2]) * (p[2] - closest_arr[2]);
        const d2 = (p[0] - 0.5) * (p[0] - 0.5) + (p[1] - 0.5) * (p[1] - 0.5) + (p[2] - 0.5) * (p[2] - 0.5);
        if (d1 < d2) {
            closest_arr = p;
        }
    }
    log(.INFO, name, "Closest Array: {}, {}, {}", .{ closest_arr[0], closest_arr[1], closest_arr[2] });

    //kf = try KalmanFilter.init(
    //    config.kalman_filter,
    //    [1][2]f32{.{ 1, 0 }}, // H
    //    [1][1]f32{.{400}}, // R
    //    null, // x
    //    [2][2]f32{ .{ 1, 0.25 }, .{ 0, 1 } }, // F
    //    [2][1]f32{ .{0.0313}, .{0.25} }, // G
    //    [2][2]f32{ .{ 500, 0 }, .{ 0, 500 } }, // P
    //    [2][2]f32{ .{ 9.765625e-6, 7.8125e-3 }, .{ 7.8125e-3, 6.25e-2 } }, // Q
    //);

    //const x = try kf.iterate(
    //    [1]f32{6.43}, // z
    //    null, // H
    //    null, // R
    //    [1]f32{39.81 - 9.81}, // u
    //    null, // F
    //    null, // G
    //    null, // Q
    //);

    //log(.INFO, name, "x_1: {d}", .{x});
    //kf.deinit();
    //ikd.deinit();

    rs_depth.module_config = .{ .dry_run = config.dry_run };
}

pub fn deinit() void {
    log(.INFO, name, "Deinitializing", .{});

    if (gpa.deinit() == .leak) {
        log(.ERROR, name, "Memory leaks detected", .{});
    }
}

test {}
