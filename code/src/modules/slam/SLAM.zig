const std = @import("std");
const log = @import("../../utils/logger.zig").log;
const Module = @import("../module.zig").Module;
const Gateway = @import("../../utils/gateway.zig").Gateway;

pub const SLAM = struct {
    const Self = @This();
    const SLAMModule = Module(SLAMGateway);
    const SLAMGateway = Gateway(RequestEnum, void);

    const RequestEnum = enum {
        START,
        STOP,
    };

    pub fn next_task(queue: SLAMModule.TaskQueue) usize {
        _ = queue;
        return 0;
    }

    pub fn start(_: void) void {
        log(.INFO, "SLAM", "Starting", .{});
    }

    pub fn stop(_: void) void {
        log(.INFO, "SLAM", "Stopping", .{});
    }

    pub fn init(name: []const u8) SLAMModule {
        log(.INFO, name, "Initializing SLAM {s}", .{"test"});
        var gateway = SLAMGateway{};
        gateway.registerMessageHandler(.START, Self.start, 1);
        gateway.registerMessageHandler(.STOP, Self.stop, 0);
        return Module(SLAMGateway).init(name, gateway, next_task);
    }

    pub fn deinit(self: *SLAM) void {
        self.message_handler.deinit();
    }

    // TODO: make this module do the realsense camera stuff in the start and stop functions
    // TODO: also try adding a kalman filter here
};
