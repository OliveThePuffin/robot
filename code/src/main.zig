const slam = @import("modules/slam/SLAM.zig").SLAM;

//const rs_depth = @import("rs-depth.zig");

pub fn main() void {

    // Initialize modules
    var module_slam = slam.init("SLAM");

    // Deinitialize modules
    module_slam.deinit();
}
