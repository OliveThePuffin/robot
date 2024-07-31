const std = @import("std");
const log = @import("../logger.zig").log;
const lrs2 = @cImport({
    @cInclude("librealsense2/rs.h");
    @cInclude("librealsense2/h/rs_pipeline.h");
});

pub const rs_module_config = struct {
    dry_run: bool = false,
    fps: u32 = 30,
    width: u32 = 640,
    height: u32 = 0,
    width_ratio: u32 = 10,
    height_ratio: u32 = 20,

    stream_index: usize = 0,
    stream: lrs2.enum_rs2_stream = lrs2.RS2_STREAM_DEPTH,
    format: lrs2.enum_rs2_format = lrs2.RS2_FORMAT_Z16,
};

const RealsenseError = error{
    NoDevice,
};

var loop_continue = false;
var loop_thread: ?std.Thread = null;
var loop_mutex: std.Thread.Mutex = .{};
pub var module_config: rs_module_config = .{};

fn check_error(e: [*c]?*lrs2.struct_rs2_error) void {
    if (e) |err| {
        std.debug.print(
            "rs_error was raised when calling {s}({s}):\n",
            .{ lrs2.rs2_get_failed_function(err.?.*), lrs2.rs2_get_failed_args(err.?.*) },
        );
        std.debug.print("    {s}\n", .{lrs2.rs2_get_error_message(err.?.*)});
        lrs2.exit(lrs2.EXIT_FAILURE);
    }
}

fn print_device_info(dev: *const lrs2.struct_rs2_device) void {
    const e: [*c]?*lrs2.struct_rs2_error = 0;
    std.debug.print(
        "\nUsing device 0, an {s}\n",
        .{lrs2.rs2_get_device_info(dev, lrs2.RS2_CAMERA_INFO_NAME, e)},
    );
    check_error(e);
    std.debug.print(
        "    Serial number: {s}\n",
        .{lrs2.rs2_get_device_info(dev, lrs2.RS2_CAMERA_INFO_SERIAL_NUMBER, e)},
    );
    check_error(e);
    std.debug.print(
        "    Firmware version: {s}\n\n",
        .{lrs2.rs2_get_device_info(dev, lrs2.RS2_CAMERA_INFO_FIRMWARE_VERSION, e)},
    );
    check_error(e);
}

fn get_depth_unit_value(dev: *const lrs2.struct_rs2_device) f32 {
    const e: [*c]?*lrs2.struct_rs2_error = 0;
    const sensor_list: ?*lrs2.struct_rs2_sensor_list = lrs2.rs2_query_sensors(dev, e);
    check_error(e);

    const num_of_sensors: c_int = lrs2.rs2_get_sensors_count(sensor_list, e);
    check_error(e);

    var depth_scale: f32 = 0;
    var is_depth_sensor_found: u1 = 0;
    for (0..@intCast(num_of_sensors)) |i| {
        const sensor = lrs2.rs2_create_sensor(sensor_list, @intCast(i), e);
        check_error(e);

        // Check if the given sensor can be extended to depth sensor interface
        is_depth_sensor_found = @intCast(lrs2.rs2_is_sensor_extendable_to(sensor, lrs2.RS2_EXTENSION_DEPTH_SENSOR, e));
        check_error(e);
        if (1 == is_depth_sensor_found) {
            depth_scale = lrs2.rs2_get_option(@ptrCast(sensor), lrs2.RS2_OPTION_DEPTH_UNITS, e);
            check_error(e);
            lrs2.rs2_delete_sensor(sensor);
            break;
        }
        lrs2.rs2_delete_sensor(sensor);
    }
    lrs2.rs2_delete_sensor_list(sensor_list);

    if (0 == is_depth_sensor_found) {
        std.debug.print("Depth sensor not found!\n", .{});
        lrs2.exit(lrs2.EXIT_FAILURE);
    }
    return depth_scale;
}

pub fn start_loop() void {
    loop_mutex.lock();
    defer loop_mutex.unlock();

    if (loop_thread) |_| {
        log(.WARN, "RealSense", "Loop already running...", .{});
    } else {
        loop_continue = true;
        loop_thread = std.Thread.spawn(.{}, depth_loop, .{}) catch unreachable;
    }
}

pub fn stop_loop() void {
    loop_mutex.lock();
    defer loop_mutex.unlock();

    if (loop_thread) |*thread| {
        loop_continue = false;
        thread.join();
        loop_thread = null;
    } else {
        log(.WARN, "RealSense", "Loop not running...", .{});
    }
}

fn depth_loop() RealsenseError!void {
    if (module_config.dry_run) {
        log(.WARN, "RealSense", "Dry run mode enabled. Skipping Depth Loop", .{});
        return;
    }

    const e: [*c]?*lrs2.struct_rs2_error = 0;

    // Create a context object. This object owns the handles to all connected realsense devices.
    // The returned object should be released with rs2_delete_context(...)
    const ctx: ?*lrs2.struct_rs2_context = lrs2.rs2_create_context(lrs2.RS2_API_VERSION, e);
    check_error(e);

    // Get a list of all the connected devices.
    // The returned object should be released with rs2_delete_device_list(...)
    const device_list: ?*lrs2.struct_rs2_device_list = lrs2.rs2_query_devices(ctx, e);
    check_error(e);

    const dev_count = lrs2.rs2_get_device_count(device_list, e);
    check_error(e);

    std.debug.print("There are {d} connected RealSense devices.\n", .{dev_count});
    if (0 == dev_count)
        return RealsenseError.NoDevice;

    // Get the first connected device
    // The returned object should be released with rs2_delete_device(...)
    const dev = if (lrs2.rs2_create_device(device_list, 0, e)) |d| d else return RealsenseError.NoDevice;
    check_error(e);

    print_device_info(dev);

    // Determine depth value corresponding to one meter
    const one_meter: u16 = @intFromFloat(1.0 / get_depth_unit_value(dev));

    // Create a pipeline to configure, start and stop camera streaming
    // The returned object should be released with rs2_delete_pipeline(...)
    const pipeline = lrs2.rs2_create_pipeline(ctx, e);
    check_error(e);

    // Create a config instance, used to specify hardware configuration
    // The returned object should be released with rs2_delete_config(...)
    const config = lrs2.rs2_create_config(e);
    check_error(e);

    // Request a specific configuration
    lrs2.rs2_config_enable_stream(
        config,
        module_config.stream,
        @intCast(module_config.stream_index),
        @intCast(module_config.width),
        @intCast(module_config.height),
        module_config.format,
        @intCast(module_config.fps),
        e,
    );
    check_error(e);

    // Start the pipeline streaming
    // The returned object should be released with rs2_delete_pipeline_profile(...)
    const pipeline_profile = lrs2.rs2_pipeline_start_with_config(pipeline, config, e);
    if (e) |_| {
        std.debug.print("The connected device doesn't support depth streaming!\n", .{});
        lrs2.exit(lrs2.EXIT_FAILURE);
    }

    const stream_profile_list = lrs2.rs2_pipeline_profile_get_streams(pipeline_profile, e);
    if (e) |_| {
        std.debug.print("Failed to create stream profile list!\n", .{});
        lrs2.exit(lrs2.EXIT_FAILURE);
    }

    const stream_profile = lrs2.rs2_get_stream_profile(stream_profile_list, 0, e);
    if (e) |_| {
        std.debug.print("Failed to create stream profile!\n");
        lrs2.exit(lrs2.EXIT_FAILURE);
    }

    var stream: lrs2.enum_rs2_stream = undefined;
    var format: lrs2.enum_rs2_format = undefined;
    var index: c_int = undefined;
    var unique_id: c_int = undefined;
    var framerate: c_int = undefined;
    lrs2.rs2_get_stream_profile_data(stream_profile, &stream, &format, &index, &unique_id, &framerate, e);
    if (e) |_| {
        std.debug.print("Failed to get stream profile data!\n");
        lrs2.exit(lrs2.EXIT_FAILURE);
    }

    var width: c_int = undefined;
    var height: c_int = undefined;
    lrs2.rs2_get_video_stream_resolution(stream_profile, &width, &height, e);
    if (e) |_| {
        std.debug.print("Failed to get video stream resolution data!\n");
        lrs2.exit(lrs2.EXIT_FAILURE);
    }
    const rows: u16 = @intCast(@divTrunc(@as(u32, @intCast(height)), module_config.height_ratio));
    const row_length: u16 = @intCast(@divTrunc(@as(u32, @intCast(width)), module_config.width_ratio));
    const display_size: u32 = (rows + 1) * (row_length + 1);

    var aa = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer aa.deinit();
    defer _ = gpa.deinit();

    const buffer: []u8 = aa.allocator().alloc(u8, display_size) catch unreachable;
    @memset(buffer, 0);
    var out: usize = undefined;

    while (loop_continue) {
        // This call waits until a new composite_frame is available
        // composite_frame holds a set of frames. It is used to prevent frame drops
        // The returned object should be released with rs2_release_frame(...)
        const frames = lrs2.rs2_pipeline_wait_for_frames(pipeline, lrs2.RS2_DEFAULT_TIMEOUT, e);
        check_error(e);

        // Returns the number of frames embedded within the composite frame
        const num_of_frames: usize = @intCast(lrs2.rs2_embedded_frames_count(frames, e));
        check_error(e);

        for (0..num_of_frames) |i| {
            // The retunred object should be released with rs2_release_frame(...)
            const frame = lrs2.rs2_extract_frame(frames, @intCast(i), e);
            check_error(e);

            // Check if the given frame can be extended to depth frame interface
            // Accept only depth frames and skip other frames
            if (0 == lrs2.rs2_is_frame_extendable_to(frame, lrs2.RS2_EXTENSION_DEPTH_FRAME, e)) {
                lrs2.rs2_release_frame(frame);
                continue;
            }

            // Retrieve depth data, configured as 16-bit depth values
            const depth_frame_data: [*]const u16 = @ptrCast(@alignCast(lrs2.rs2_get_frame_data(frame, e)));
            check_error(e);

            // Print a simple text-based representation of the image
            // by breaking it into 10x5 pixel regions and approximating the coverage of pixels within one meter
            out = 0;
            const coverage: []usize = gpa.allocator().alloc(usize, row_length) catch unreachable;
            defer gpa.allocator().free(coverage);
            @memset(coverage, 0);

            for (0..@intCast(height)) |y| {
                for (0..@intCast(width)) |x| {
                    // Create a depth histogram to each row
                    const coverage_index: usize = @min(row_length - 1, @divTrunc(x, module_config.width_ratio));
                    const depth: u16 = depth_frame_data[y * @as(usize, @intCast(width)) + x];
                    if (depth > 0 and depth < one_meter) {
                        coverage[coverage_index] += 1;
                    }
                }

                if ((y % module_config.height_ratio) == (module_config.height_ratio - 1)) {
                    for (0..row_length) |j| {
                        const pixels: []const u8 = " .:nhBXWW";
                        //const pixels: []const u8 = " ░▒▓█";
                        const pixel_index: usize = @min(pixels.len - 1, @divTrunc(
                            coverage[j],
                            @divTrunc(module_config.height_ratio * module_config.width_ratio, pixels.len - 1),
                        ));
                        buffer[out] = pixels[pixel_index];
                        out += 1;
                        coverage[j] = 0;
                    }
                    buffer[out] = '\n';
                    out += 1;
                }
            }
            buffer[out] = 0;
            out += 1;
            std.debug.print("\n{s}", .{buffer});

            lrs2.rs2_release_frame(frame);
        }
        lrs2.rs2_release_frame(frames);
    }

    // Stop the pipeline streaming
    lrs2.rs2_pipeline_stop(pipeline, e);
    check_error(e);

    // Release resources
    lrs2.rs2_delete_pipeline_profile(pipeline_profile);
    //lrs2.rs2_delete_stream_profiles_list(stream_profile_list);
    //lrs2.rs2_delete_stream_profile(stream_profile);
    lrs2.rs2_delete_config(config);
    lrs2.rs2_delete_pipeline(pipeline);
    lrs2.rs2_delete_device(dev);
    lrs2.rs2_delete_device_list(device_list);
    lrs2.rs2_delete_context(ctx);
}
