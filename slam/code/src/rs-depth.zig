const std = @import("std");
const lrs2 = @cImport({
    @cDefine("STREAM", "RS2_STREAM_DEPTH");
    @cDefine("FORMAT", "RS2_FORMAT_Z16");
    @cDefine("WIDTH", "640");
    @cDefine("HEIGHT", "0");
    @cDefine("FPS", "30");
    @cDefine("STREAM_INDEX", "0");
    @cDefine("HEIGHT_RATIO", "20");
    @cDefine("WIDTH_RATIO", "10");

    @cInclude("librealsense2/rs.h");
    @cInclude("librealsense2/h/rs_pipeline.h");
    @cInclude("librealsense2/h/rs_option.h");
    @cInclude("librealsense2/h/rs_frame.h");
    @cInclude("example.h");
});

fn check_error(e: [*c]?*lrs2.struct_rs2_error) void {
    if (e) |err| {
        std.debug.print("rs_error was raised when calling {s}({s}):\n", .{ lrs2.rs2_get_failed_function(err.?.*), lrs2.rs2_get_failed_args(err.?.*) });
        std.debug.print("    {s}\n", .{lrs2.rs2_get_error_message(err.?.*)});
        lrs2.exit(lrs2.EXIT_FAILURE);
    }
}

fn print_device_info(dev: *const lrs2.struct_rs2_device) void {
    const e: [*c]?*lrs2.struct_rs2_error = 0;
    std.debug.print("\nUsing device 0, an {s}\n", .{lrs2.rs2_get_device_info(dev, lrs2.RS2_CAMERA_INFO_NAME, e)});
    check_error(e);
    std.debug.print("    Serial number: {s}\n", .{lrs2.rs2_get_device_info(dev, lrs2.RS2_CAMERA_INFO_SERIAL_NUMBER, e)});
    check_error(e);
    std.debug.print("    Firmware version: {s}\n\n", .{lrs2.rs2_get_device_info(dev, lrs2.RS2_CAMERA_INFO_FIRMWARE_VERSION, e)});
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

pub fn depth_loop() c_int {
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
        return lrs2.EXIT_FAILURE;

    // Get the first connected device
    // The returned object should be released with rs2_delete_device(...)
    const dev = if (lrs2.rs2_create_device(device_list, 0, e)) |d| d else return lrs2.EXIT_FAILURE;
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
    lrs2.rs2_config_enable_stream(config, lrs2.STREAM, lrs2.STREAM_INDEX, lrs2.WIDTH, lrs2.HEIGHT, lrs2.FORMAT, lrs2.FPS, e);
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
    const rows: u16 = @intCast(@divTrunc(height, lrs2.HEIGHT_RATIO));
    const row_length: u16 = @intCast(@divTrunc(width, lrs2.WIDTH_RATIO));
    const display_size: u32 = (rows + 1) * (row_length + 1);

    var aa = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer aa.deinit();
    defer gpa.deinit();

    const buffer: []u8 = aa.allocator().alloc(u8, display_size) catch unreachable;
    @memset(buffer, 0);
    var out: usize = undefined;

    while (true) {
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

            // Print a simple text-based representation of the image, by breaking it into 10x5 pixel regions and approximating the coverage of pixels within one meter
            out = 0;
            const coverage: []usize = gpa.allocator().alloc(usize, row_length) catch unreachable;
            defer gpa.allocator().free(coverage);
            @memset(coverage, 0);

            for (0..@intCast(height)) |y| {
                for (0..@intCast(width)) |x| {
                    // Create a depth histogram to each row
                    const coverage_index: usize = @divTrunc(x, lrs2.WIDTH_RATIO);
                    const depth: u16 = depth_frame_data[y * @as(usize, @intCast(width)) + x];
                    if (depth > 0 and depth < one_meter) {
                        coverage[coverage_index] += 1;
                    }
                }

                if ((y % lrs2.HEIGHT_RATIO) == (lrs2.HEIGHT_RATIO - 1)) {
                    for (0..row_length) |j| {
                        const pixels: []const u8 = " .:nhBXWW";
                        const pixel_index: usize = @divTrunc(coverage[j], @divTrunc(lrs2.HEIGHT_RATIO * lrs2.WIDTH_RATIO, pixels.len - 1));
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

    return lrs2.EXIT_SUCCESS;
}
