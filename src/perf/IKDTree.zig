const std = @import("std");
const Log = @import("Log").Log;
const IKDTree = @import("../slam/IKDTree.zig").IKDTree;

pub fn ikd_tree_perf() !void {
    var log = try Log.init(.{
        .level = .DEBUG,
        .abs_path = "/tmp/CLAW/perf",
        .channel = "IKDTree",
    });
    defer log.deinit();
    const I3DTree = IKDTree(3);
    var aa = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer aa.deinit();
    const allocator = aa.allocator();
    const tol = 0.0001;

    const config = I3DTree.Config{
        .a_bal = 0.75,
        .a_del = 0.5,
        .relative_tolrance = tol,
    };

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
    defer allocator.free(points);

    // Making a fixed buffer allocator makes the ikd tree much faster
    const buffer = try allocator.alloc(u8, 1 * try std.math.powi(usize, 2, 30));
    defer allocator.free(buffer);
    var fba = std.heap.FixedBufferAllocator.init(buffer);

    var timer = std.time.Timer.start() catch unreachable;
    var ikd = try I3DTree.init(points, config, fba.allocator(), &log);
    defer ikd.deinit();

    log.info("IKD tree {} nodes init time {}", .{ dim, std.fmt.fmtDuration(timer.lap()) });

    _ = timer.lap();
    const query_point: [3]f32 = .{ 3, 3, 3 };
    const nearest = ikd.nearestNeighbor(query_point).?;

    log.info("IKD tree nnSearch time {}", .{std.fmt.fmtDuration(timer.lap())});
    log.debug("Closest point to {d}: {d}", .{ query_point, nearest });

    _ = timer.lap();
    const knearest = try ikd.kNearestNeighbors(query_point, 5);

    log.info("IKD tree knnSearch time {}", .{std.fmt.fmtDuration(timer.lap())});
    log.debug("Closest points to {d}:", .{query_point});
    for (knearest) |k| {
        log.debug("  {d}", .{k});
    }
}
