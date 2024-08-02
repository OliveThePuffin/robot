const std = @import("std");

// module utils
const logger = @import("logger");

// modules
const Slam = @import("Slam");
const OpenCL = @import("OpenCL");

test {
    // utils
    _ = logger;

    // modules
    _ = Slam;
    _ = OpenCL;
}
