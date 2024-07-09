const std = @import("std");

// utils
const gateway = @import("utils/gateway.zig");
const logger = @import("utils/logger.zig");

// modules
const modules = @import("modules/module.zig");
const slam = @import("modules/slam/slam.zig");

test {
    // utils
    _ = gateway;
    _ = logger;

    // modules
    _ = modules;
    _ = slam;
}
