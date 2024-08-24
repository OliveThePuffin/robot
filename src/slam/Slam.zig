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
    var rand_default_prng = std.Random.DefaultPrng.init(0);
    const rand = rand_default_prng.random();

    const dim = 100000;
    var points = try allocator.alloc([3]f32, dim);
    for (0..dim) |k| {
        points[k] = .{
            @round(rand.float(f32) * 500) / 100,
            @round(rand.float(f32) * 500) / 100,
            @round(rand.float(f32) * 500) / 100,
        };
    }

    // Making a fixed buffer allocator makes the ikd tree much faster
    const buffer = try allocator.alloc(u8, 1 * try std.math.powi(usize, 2, 30));
    defer allocator.free(buffer);
    var fba = std.heap.FixedBufferAllocator.init(buffer);

    var timer = std.time.Timer.start() catch unreachable;
    var ikd = try I3DTree.init(points, config.ikd_tree, fba.allocator());
    log(.DEBUG, name, "IKD tree init time {}", .{std.fmt.fmtDuration(timer.lap())});

    allocator.free(points);
    defer ikd.deinit();
    //ikd.print();

    const query_point: [3]f32 = .{ 3, 3, 3 };
    _ = timer.lap();
    const nearest = ikd.nearestNeighbor(query_point).?;
    log(.DEBUG, name, "IKD tree nnSearch time {}", .{std.fmt.fmtDuration(timer.lap())});
    log(.INFO, name, "Closest point to {d}: {d}", .{ query_point, nearest });

    _ = timer.lap();
    const knearest = try ikd.kNearestNeighbors(query_point, 5);
    log(.DEBUG, name, "IKD tree knnSearch time {}", .{std.fmt.fmtDuration(timer.lap())});
    log(.INFO, name, "Closest points to {d}:", .{query_point});
    for (knearest) |k| {
        log(.INFO, name, "  {d}", .{k});
    }

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
    //defer kf.deinit();

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

    rs_depth.module_config = .{ .dry_run = config.dry_run };
}

pub fn deinit() void {
    log(.INFO, name, "Deinitializing", .{});

    if (gpa.deinit() == .leak) {
        log(.ERROR, name, "Memory leaks detected", .{});
    }
}

test {}
