const Log = @import("Log").Log;
const LogConfig = @import("Log").Config;
const lrs2 = @cImport({
    @cInclude("librealsense2/rs.h");
    @cInclude("librealsense2/h/rs_pipeline.h");
});

var log: Log = undefined;
var dry_run = false;
var realsense_context: ?*lrs2.struct_rs2_context = null;
var realsense_pipeline: ?*lrs2.struct_rs2_pipeline = null;
var realsense_config: ?*lrs2.struct_rs2_config = null;
var realsense_pipeline_profile: ?*lrs2.struct_rs2_pipeline_profile = null;

pub const Config = struct {
    log: LogConfig,
    dry_run: bool,
};

const RealSenseError = error{
    NoDevice,
    NoSensor,
    NoIMU,
};

pub fn init(config: Config) !void {
    dry_run = config.dry_run;
    log = try Log.init(config.log);
    log.info("Initializing", .{});
    if (dry_run) {
        log.warn("Dry run mode", .{});
        return;
    }

    const e: [*c]?*lrs2.struct_rs2_error = 0;

    realsense_context = lrs2.rs2_create_context(lrs2.RS2_API_VERSION, e);
    checkError(e);
    errdefer lrs2.rs2_delete_context(realsense_context);

    realsense_pipeline = lrs2.rs2_create_pipeline(realsense_context, e);
    checkError(e);
    errdefer lrs2.rs2_delete_pipeline(realsense_pipeline);

    realsense_config = lrs2.rs2_create_config(e);
    checkError(e);
    errdefer lrs2.rs2_delete_config(realsense_config);

    try findIMU();
}

pub fn start() void {
    log.info("Starting", .{});
    if (dry_run) {
        return;
    }
    const e: [*c]?*lrs2.struct_rs2_error = 0;
    realsense_pipeline_profile = lrs2.rs2_pipeline_start_with_config_and_callback(
        realsense_pipeline,
        realsense_config,
        frame_callback,
        null,
        e,
    );
    checkError(e);
}

pub fn stop() void {
    log.info("Stopping", .{});
    if (dry_run) {
        return;
    }
    const e: [*c]?*lrs2.struct_rs2_error = 0;
    lrs2.rs2_pipeline_stop(realsense_pipeline, e);
    checkError(e);
}

fn frame_callback(frame: ?*lrs2.struct_rs2_frame, user: ?*anyopaque) callconv(.C) void {
    _ = user;

    const e: [*c]?*lrs2.struct_rs2_error = 0;

    const profile = lrs2.rs2_get_frame_stream_profile(frame, e);
    checkError(e);

    var stream: lrs2.enum_rs2_stream = undefined;
    var format: lrs2.enum_rs2_format = undefined;
    var index: c_int = undefined;
    var unique_id: c_int = undefined;
    var framerate: c_int = undefined;
    lrs2.rs2_get_stream_profile_data(profile, &stream, &format, &index, &unique_id, &framerate, e);
    checkError(e);

    if (frame != null and stream == lrs2.RS2_STREAM_GYRO and format == lrs2.RS2_FORMAT_MOTION_XYZ32F) {
        const timestamp = lrs2.rs2_get_frame_timestamp(frame, e);
        checkError(e);
        const gyro_data = @as(?*lrs2.rs2_vector, @constCast(@alignCast(@ptrCast(lrs2.rs2_get_frame_data(frame, e)))));
        checkError(e);
        if (gyro_data) |gyro| {
            accumulateGyro(gyro, timestamp);
        }
    } else if (frame != null and stream == lrs2.RS2_STREAM_ACCEL and format == lrs2.RS2_FORMAT_MOTION_XYZ32F) {
        const timestamp = lrs2.rs2_get_frame_timestamp(frame, e);
        checkError(e);
        const accel_data = @as(?*lrs2.rs2_vector, @constCast(@alignCast(@ptrCast(lrs2.rs2_get_frame_data(frame, e)))));
        checkError(e);
        if (accel_data) |accel| {
            accumulateAccel(accel, timestamp);
        }
    } else {}
    lrs2.rs2_release_frame(frame);
}

pub fn deinit() void {
    lrs2.rs2_delete_config(realsense_config);
    lrs2.rs2_delete_pipeline(realsense_pipeline);
    lrs2.rs2_delete_context(realsense_context);
    log.deinit();
}

fn checkError(e: [*c]?*lrs2.struct_rs2_error) void {
    if (e) |err| {
        log.err("realsense_error was raised when calling {s}({s}):", .{
            lrs2.rs2_get_failed_function(err.?.*),
            lrs2.rs2_get_failed_args(err.?.*),
        });
        log.err("    {s}", .{lrs2.rs2_get_error_message(err.?.*)});
        lrs2.exit(lrs2.EXIT_FAILURE);
    }
}

fn findIMU() RealSenseError!void {
    var found_gyro = false;
    var found_accel = false;

    const e: [*c]?*lrs2.struct_rs2_error = 0;

    const device_list: ?*lrs2.struct_rs2_device_list = lrs2.rs2_query_devices(realsense_context, e);
    checkError(e);
    defer lrs2.rs2_delete_device_list(device_list);

    const device_count = lrs2.rs2_get_device_count(device_list, e);
    checkError(e);

    log.debug("There are {d} connected Realsense devices.", .{device_count});
    if (device_count == 0)
        return RealSenseError.NoDevice;

    checkError(e);

    for (0..@intCast(device_count)) |device_index| {
        const device = lrs2.rs2_create_device(device_list, @intCast(device_index), e);
        checkError(e);
        defer lrs2.rs2_delete_device(device);
        log.debug("Device {d}: Name: {s}: Serial number: {s}, Firmware version: {s}", .{
            device_index,
            lrs2.rs2_get_device_info(device, lrs2.RS2_CAMERA_INFO_NAME, e),
            lrs2.rs2_get_device_info(device, lrs2.RS2_CAMERA_INFO_SERIAL_NUMBER, e),
            lrs2.rs2_get_device_info(device, lrs2.RS2_CAMERA_INFO_FIRMWARE_VERSION, e),
        });
        checkError(e);

        const sensor_list = lrs2.rs2_query_sensors(device, e);
        checkError(e);
        defer lrs2.rs2_delete_sensor_list(sensor_list);

        const sensor_count = lrs2.rs2_get_sensors_count(sensor_list, e);
        checkError(e);
        log.debug("Device {d} has {d} sensors.", .{ device_index, sensor_count });

        found_gyro = found_gyro and found_accel;
        found_accel = found_gyro and found_accel;
        for (0..@intCast(sensor_count)) |sensor_index| {
            const sensor = lrs2.rs2_create_sensor(sensor_list, @intCast(sensor_index), e);
            checkError(e);
            defer lrs2.rs2_delete_sensor(sensor);
            log.debug("    Sensor {d}: {s}", .{
                sensor_index,
                lrs2.rs2_get_sensor_info(sensor, lrs2.RS2_CAMERA_INFO_NAME, e),
            });
            checkError(e);

            const profile_list = lrs2.rs2_get_stream_profiles(sensor, e);
            checkError(e);
            const profile_count = lrs2.rs2_get_stream_profiles_count(profile_list, e);
            checkError(e);
            log.debug("    Sensor {d} has {d} stream profiles.", .{ sensor_index, profile_count });

            for (0..@intCast(profile_count)) |profile_index| {
                const profile = lrs2.rs2_get_stream_profile(profile_list, @intCast(profile_index), e);
                checkError(e);

                var stream: lrs2.enum_rs2_stream = undefined;
                var format: lrs2.enum_rs2_format = undefined;
                var index: c_int = undefined;
                var unique_id: c_int = undefined;
                var framerate: c_int = undefined;
                lrs2.rs2_get_stream_profile_data(profile, &stream, &format, &index, &unique_id, &framerate, e);
                checkError(e);

                var width: c_int = 0;
                var height: c_int = 0;
                lrs2.rs2_get_video_stream_resolution(profile, &width, &height, e);
                checkError(e);

                log.debug("        Profile {d}: Stream: {s}, Format: {s}, Width: {d}, Height: {d}, Framerate: {d}", .{
                    profile_index,
                    lrs2.rs2_stream_to_string(stream),
                    lrs2.rs2_format_to_string(format),
                    width,
                    height,
                    framerate,
                });

                switch (stream) {
                    lrs2.RS2_STREAM_GYRO => found_gyro = true,
                    lrs2.RS2_STREAM_ACCEL => found_accel = true,
                    else => {},
                }
            }
        }
    }
    if (!found_gyro or !found_accel) {
        return RealSenseError.NoIMU;
    }

    lrs2.rs2_config_enable_stream(realsense_config, lrs2.RS2_STREAM_GYRO, -1, 0, 0, lrs2.RS2_FORMAT_MOTION_XYZ32F, 0, e);
    checkError(e);
    lrs2.rs2_config_enable_stream(realsense_config, lrs2.RS2_STREAM_ACCEL, -1, 0, 0, lrs2.RS2_FORMAT_MOTION_XYZ32F, 0, e);
    checkError(e);
}

fn accumulateGyro(gyro_data: *lrs2.rs2_vector, timestamp: lrs2.rs2_time_t) void {
    // TODO: accumulate gyro data and delete this
    log.debug("Gyro data: ({d:2.3}, {d:2.3}, {d:2.3}) | timestamp: {d}", .{
        gyro_data.x,
        gyro_data.y,
        gyro_data.z,
        timestamp / 1000,
    });
}

fn accumulateAccel(accel_data: *lrs2.rs2_vector, timestamp: lrs2.rs2_time_t) void {
    // TODO: accumulate accel data and delete this
    log.debug("Accel data: ({d:2.3}, {d:2.3}, {d:2.3}) | timestamp: {d}", .{
        accel_data.x,
        accel_data.y,
        accel_data.z,
        timestamp / 1000,
    });
}
