const std = @import("std");

// module utils
const gateway = @import("modules/gateway.zig");
const logger = @import("modules/logger.zig");
const modules = @import("modules/Module.zig");

// modules
const slam = @import("modules/slam/Slam.zig");

test {
    // utils
    _ = gateway;
    _ = logger;

    // modules
    _ = modules;
    _ = slam;
}
