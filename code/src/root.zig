const std = @import("std");

// module utils
const gateway = @import("modules/gateway.zig");
const logger = @import("modules/logger.zig");
const modules = @import("modules/module.zig");

// modules
const slam = @import("modules/slam/slam.zig");

test {
    // utils
    _ = gateway;
    _ = logger;

    // modules
    _ = modules;
    _ = slam;
}
