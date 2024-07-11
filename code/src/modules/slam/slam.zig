const std = @import("std");
const log = @import("../logger.zig").log;
const Module = @import("../module.zig").Module;
const Gateway = @import("../gateway.zig").Gateway;

pub const Slam = struct {
    const Self = @This();
    const SlamModule = Module(SlamGateway, SlamTaskQueue);
    const SlamGateway = Gateway(RequestEnum, void);
    const SlamTaskQueue = std.PriorityQueue(SlamGateway.Task, void, greaterThan);

    pub const RequestEnum = enum {
        START,
        STOP,
    };

    fn start(_: void) void {
        log(.INFO, "Slam", "Starting", .{});
    }

    fn stop(_: void) void {
        log(.INFO, "Slam", "Stopping", .{});
    }

    fn greaterThan(context: void, a: SlamGateway.Task, b: SlamGateway.Task) std.math.Order {
        _ = context;
        return std.math.order(b.priority, a.priority);
    }

    pub fn init(name: []const u8, allocator: std.mem.Allocator) !*SlamModule {
        var gateway = SlamGateway{};
        gateway.registerMessageHandler(.START, Self.start, 1);
        gateway.registerMessageHandler(.STOP, Self.stop, 0);

        return SlamModule.init(name, gateway, SlamTaskQueue.init(allocator, {}), allocator);
    }

    // TODO: make this module do the realsense camera stuff in the start and stop functions
    // TODO: also try adding a kalman filter here
};
