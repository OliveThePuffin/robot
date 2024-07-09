const Slam = @import("modules/slam/slam.zig").Slam;

//const rs_depth = @import("rs-depth.zig");

pub fn main() void {

    // Initialize modules
    var module_slam = Slam.init("SLAM");

    // Deinitialize modules
    module_slam.deinit();
}
