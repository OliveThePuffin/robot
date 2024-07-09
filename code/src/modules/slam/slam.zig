const std = @import("std");
const log = @import("../../utils/logger.zig").log;
const Module = @import("../module.zig").Module;
const Gateway = @import("../../utils/gateway.zig").Gateway;

pub const Slam = struct {
    const Self = @This();
    const SlamModule = Module(SlamGateway);
    const SlamGateway = Gateway(RequestEnum, void);

    const RequestEnum = enum {
        START,
        STOP,
    };

    fn next_task(queue: SlamModule.TaskQueue) usize {
        _ = queue;
        return 0;
    }

    fn start(_: void) void {
        log(.INFO, "Slam", "Starting", .{});
    }

    fn stop(_: void) void {
        log(.INFO, "Slam", "Stopping", .{});
    }

    pub fn init(name: []const u8) SlamModule {
        var gateway = SlamGateway{};
        gateway.registerMessageHandler(.START, Self.start, 1);
        gateway.registerMessageHandler(.STOP, Self.stop, 0);
        return Module(SlamGateway).init(name, gateway, next_task);
    }

    pub fn deinit(self: *Slam) void {
        self.message_handler.deinit();
    }

    // TODO: make this module do the realsense camera stuff in the start and stop functions
    // TODO: also try adding a kalman filter here
};
