const std = @import("std");
const Log = @import("Log").Log;
const clh = @import("ocl_helper");
const cl = clh.cl;

// x: estimated state vector                n_x * 1
// z: measurements vector                   n_z * 1
// u: input vector                          n_u * 1
// P: estimated state covariance matrix     n_x * n_x

// F: state transition matrix               n_x * n_x
// G: control matrix                        n_x * n_u
// Q: process noise covariance matrix       n_x * n_x
// R: measurement noise covariance matrix   n_z * n_z

// w: process noise vector                  n_x * 1 (unknowable, implicitly added to x)
// v: measurement noise vector              n_z * 1 (unknowable, implicitly added to z)
// H: observation matrix                    n_z * n_x
// K: Kalman gain matrix                    n_x * n_z
//
// n_x: state dimension
// n_z: measurement dimension
// n_u: input/control dimension

// TODO: Implement FDI
// Fault Detection and Isolation (FDI):
//    Implement an FDI algorithm that monitors the residuals (i.e., the difference between the predicted measurement and the actual measurement) for each sensor.
//    If a sensor's residuals consistently exceed a certain threshold, the algorithm could either flag the sensor as faulty or adjust the Kalman filter to reduce trust in that sensor.
//    This can be done by increasing the R value for that sensor or even excluding its data from the update.

pub fn KalmanFilter(comptime state_dim: u32, comptime measure_dim: u32, comptime control_dim: u32) type {
    const size_type = @sizeOf(f32);
    const size_x = state_dim * size_type;
    const size_z = measure_dim * size_type;
    const size_u = control_dim * size_type;
    const size_F = state_dim * state_dim * size_type;
    const size_G = state_dim * control_dim * size_type;
    const size_P = state_dim * state_dim * size_type;
    const size_Q = state_dim * state_dim * size_type;
    const size_R = measure_dim * measure_dim * size_type;
    const size_H = measure_dim * state_dim * size_type;
    const size_K = state_dim * measure_dim * size_type;
    const size_PHT = state_dim * measure_dim * size_type;
    const size_HPHTR = measure_dim * measure_dim * size_type;
    const size_IKH = state_dim * state_dim * size_type;
    const size_IKHP = state_dim * state_dim * size_type;
    const size_IKHPIKHT = state_dim * state_dim * size_type;
    const size_KR = state_dim * measure_dim * size_type;
    return struct {
        const Self = @This();
        const name = "KalmanFilter";

        pub const Config = struct {
            opencl: clh.Config,
        };

        const ocl_src = @embedFile("kalman_filter.cl");

        const Stage = enum(u8) {
            PREDICT_STATE,
            PREDICT_COVARIANCE1,
            PREDICT_COVARIANCE2,
            UPDATE_GAIN1,
            UPDATE_GAIN2,
            UPDATE_GAIN3,
            UPDATE_GAIN4,
            UPDATE_GAIN5,
            UPDATE_STATE,
            UPDATE_COVARIANCE1,
            UPDATE_COVARIANCE2,
            UPDATE_COVARIANCE3,
            UPDATE_COVARIANCE4,
            UPDATE_COVARIANCE5,
        };
        const stage_count = @typeInfo(Stage).Enum.fields.len;

        const OpenCLData = struct {
            allocator: std.mem.Allocator,
            platform_id: cl.platform.cl_platform_id,
            devices: []cl.device.cl_device_id,
            device_id: cl.device.cl_device_id,
            context: cl.context.cl_context,

            kernels: [stage_count]cl.kernel.cl_kernel,
            program: cl.program.cl_program,
            queue_state: cl.command_queue.cl_command_queue,
            queue_covariance: cl.command_queue.cl_command_queue,

            buffer_x: cl.buffer.cl_mem,
            buffer_z: cl.buffer.cl_mem,
            buffer_u: cl.buffer.cl_mem,

            buffer_F: cl.buffer.cl_mem,
            buffer_G: cl.buffer.cl_mem,
            buffer_P: cl.buffer.cl_mem,
            buffer_Q: cl.buffer.cl_mem,
            buffer_R: cl.buffer.cl_mem,

            buffer_H: cl.buffer.cl_mem,
            buffer_K: cl.buffer.cl_mem,

            // Intermediate buffers
            buffer_PHT: cl.buffer.cl_mem,
            buffer_HPHTR: cl.buffer.cl_mem,
            buffer_HPHTRL: cl.buffer.cl_mem,
            buffer_HPHTRU: cl.buffer.cl_mem,
            buffer_HPHTRI: cl.buffer.cl_mem,

            buffer_IKH: cl.buffer.cl_mem,
            buffer_IKHP: cl.buffer.cl_mem,
            buffer_IKHPIKHT: cl.buffer.cl_mem,
            buffer_KR: cl.buffer.cl_mem,

            pub fn init(config: clh.Config, allocator: std.mem.Allocator, log: *Log) !OpenCLData {
                const platform_id = try clh.choosePlatform(config.platform_name, allocator, log);
                const devices = try clh.getDevices(platform_id, allocator);
                const device_id = try clh.chooseDevice(devices, config.device_name, allocator, log);
                const context = try cl.context.create(null, devices, null, null);

                const queue_state = try cl.command_queue.create(context, device_id, 0);
                const queue_covariance = try cl.command_queue.create(context, device_id, 0);
                const program = try clh.compileOpenclSrc(&[_][]const u8{ocl_src}, allocator, context, device_id, log);

                var kernels: [stage_count]cl.kernel.cl_kernel = undefined;
                inline for (@typeInfo(Stage).Enum.fields, 0..) |stage, i| {
                    log.debug("Creating kernel: {s}", .{stage.name});
                    kernels[i] = try cl.kernel.create(program, stage.name);
                }

                const ro_flag = @intFromEnum(cl.buffer.enums.mem_flags.read_only);
                const rw_flag = @intFromEnum(cl.buffer.enums.mem_flags.read_write);

                // buffers
                const buffer_x = try cl.buffer.create(context, rw_flag, size_x, null);
                errdefer cl.buffer.release(buffer_x) catch unreachable;
                const buffer_z = try cl.buffer.create(context, ro_flag, size_z, null);
                errdefer cl.buffer.release(buffer_z) catch unreachable;
                const buffer_u = try cl.buffer.create(context, ro_flag, control_dim * @sizeOf(f32), null);
                errdefer cl.buffer.release(buffer_u) catch unreachable;
                const buffer_F = try cl.buffer.create(context, ro_flag, size_F, null);
                errdefer cl.buffer.release(buffer_F) catch unreachable;
                const buffer_G = try cl.buffer.create(context, ro_flag, state_dim * control_dim * @sizeOf(f32), null);
                errdefer cl.buffer.release(buffer_G) catch unreachable;
                const buffer_P = try cl.buffer.create(context, rw_flag, size_P, null);
                errdefer cl.buffer.release(buffer_P) catch unreachable;
                const buffer_Q = try cl.buffer.create(context, ro_flag, size_Q, null);
                errdefer cl.buffer.release(buffer_Q) catch unreachable;
                const buffer_R = try cl.buffer.create(context, ro_flag, size_R, null);
                errdefer cl.buffer.release(buffer_R) catch unreachable;
                const buffer_H = try cl.buffer.create(context, ro_flag, size_H, null);
                errdefer cl.buffer.release(buffer_H) catch unreachable;
                const buffer_K = try cl.buffer.create(context, rw_flag, size_K, null);
                errdefer cl.buffer.release(buffer_K) catch unreachable;

                // intermediate buffers
                const buffer_PHT = try cl.buffer.create(context, rw_flag, size_PHT, null);
                errdefer cl.buffer.release(buffer_PHT) catch unreachable;
                const buffer_HPHTR = try cl.buffer.create(context, rw_flag, size_HPHTR, null);
                errdefer cl.buffer.release(buffer_HPHTR) catch unreachable;
                const buffer_HPHTRL = try cl.buffer.create(context, rw_flag, size_HPHTR, null);
                errdefer cl.buffer.release(buffer_HPHTRL) catch unreachable;
                const buffer_HPHTRU = try cl.buffer.create(context, rw_flag, size_HPHTR, null);
                errdefer cl.buffer.release(buffer_HPHTRU) catch unreachable;
                const buffer_HPHTRI = try cl.buffer.create(context, rw_flag, size_HPHTR, null);
                errdefer cl.buffer.release(buffer_HPHTRI) catch unreachable;
                const buffer_IKH = try cl.buffer.create(context, rw_flag, size_IKH, null);
                errdefer cl.buffer.release(buffer_IKH) catch unreachable;
                const buffer_IKHP = try cl.buffer.create(context, rw_flag, size_IKHP, null);
                errdefer cl.buffer.release(buffer_IKHP) catch unreachable;
                const buffer_IKHPIKHT = try cl.buffer.create(context, rw_flag, size_IKHPIKHT, null);
                errdefer cl.buffer.release(buffer_IKHPIKHT) catch unreachable;
                const buffer_KR = try cl.buffer.create(context, rw_flag, size_KR, null);
                errdefer cl.buffer.release(buffer_KR) catch unreachable;

                const size_buffer = @sizeOf(cl.buffer.cl_mem);
                const predictStateId = @intFromEnum(Stage.PREDICT_STATE);
                const predictCovariance1Id = @intFromEnum(Stage.PREDICT_COVARIANCE1);
                const predictCovariance2Id = @intFromEnum(Stage.PREDICT_COVARIANCE2);
                const updateGain1Id = @intFromEnum(Stage.UPDATE_GAIN1);
                const updateGain2Id = @intFromEnum(Stage.UPDATE_GAIN2);
                const updateGain3Id = @intFromEnum(Stage.UPDATE_GAIN3);
                const updateGain4Id = @intFromEnum(Stage.UPDATE_GAIN4);
                const updateGain5Id = @intFromEnum(Stage.UPDATE_GAIN5);
                const updateStateId = @intFromEnum(Stage.UPDATE_STATE);
                const updateCovariance1Id = @intFromEnum(Stage.UPDATE_COVARIANCE1);
                const updateCovariance2Id = @intFromEnum(Stage.UPDATE_COVARIANCE2);
                const updateCovariance3Id = @intFromEnum(Stage.UPDATE_COVARIANCE3);
                const updateCovariance4Id = @intFromEnum(Stage.UPDATE_COVARIANCE4);
                const updateCovariance5Id = @intFromEnum(Stage.UPDATE_COVARIANCE5);

                try cl.kernel.set_arg(kernels[predictStateId], 0, size_buffer, @ptrCast(&buffer_x));
                try cl.kernel.set_arg(kernels[predictStateId], 1, size_buffer, @ptrCast(&buffer_u));
                try cl.kernel.set_arg(kernels[predictStateId], 2, size_buffer, @ptrCast(&buffer_F));
                try cl.kernel.set_arg(kernels[predictStateId], 3, size_buffer, @ptrCast(&buffer_G));
                try cl.kernel.set_arg(kernels[predictStateId], 4, @sizeOf(u32), &state_dim);
                try cl.kernel.set_arg(kernels[predictStateId], 5, @sizeOf(u32), &control_dim);

                try cl.kernel.set_arg(kernels[predictCovariance1Id], 0, size_buffer, @ptrCast(&buffer_P));
                try cl.kernel.set_arg(kernels[predictCovariance1Id], 1, size_buffer, @ptrCast(&buffer_F));
                try cl.kernel.set_arg(kernels[predictCovariance1Id], 2, @sizeOf(u32), &state_dim);

                try cl.kernel.set_arg(kernels[predictCovariance2Id], 0, size_buffer, @ptrCast(&buffer_P));
                try cl.kernel.set_arg(kernels[predictCovariance2Id], 1, size_buffer, @ptrCast(&buffer_F));
                try cl.kernel.set_arg(kernels[predictCovariance2Id], 2, size_buffer, @ptrCast(&buffer_Q));
                try cl.kernel.set_arg(kernels[predictCovariance2Id], 3, @sizeOf(u32), &state_dim);

                try cl.kernel.set_arg(kernels[updateGain1Id], 0, size_buffer, @ptrCast(&buffer_PHT));
                try cl.kernel.set_arg(kernels[updateGain1Id], 1, size_buffer, @ptrCast(&buffer_P));
                try cl.kernel.set_arg(kernels[updateGain1Id], 2, size_buffer, @ptrCast(&buffer_H));
                try cl.kernel.set_arg(kernels[updateGain1Id], 3, @sizeOf(u32), &state_dim);
                try cl.kernel.set_arg(kernels[updateGain1Id], 4, @sizeOf(u32), &measure_dim);

                try cl.kernel.set_arg(kernels[updateGain2Id], 0, size_buffer, @ptrCast(&buffer_HPHTR));
                try cl.kernel.set_arg(kernels[updateGain2Id], 1, size_buffer, @ptrCast(&buffer_PHT));
                try cl.kernel.set_arg(kernels[updateGain2Id], 2, size_buffer, @ptrCast(&buffer_H));
                try cl.kernel.set_arg(kernels[updateGain2Id], 3, size_buffer, @ptrCast(&buffer_R));
                try cl.kernel.set_arg(kernels[updateGain2Id], 4, @sizeOf(u32), &state_dim);
                try cl.kernel.set_arg(kernels[updateGain2Id], 5, @sizeOf(u32), &measure_dim);

                try cl.kernel.set_arg(kernels[updateGain3Id], 0, size_buffer, @ptrCast(&buffer_HPHTRL));
                try cl.kernel.set_arg(kernels[updateGain3Id], 1, size_buffer, @ptrCast(&buffer_HPHTRU));
                try cl.kernel.set_arg(kernels[updateGain3Id], 2, size_buffer, @ptrCast(&buffer_HPHTR));
                try cl.kernel.set_arg(kernels[updateGain3Id], 3, @sizeOf(u32), &measure_dim);

                try cl.kernel.set_arg(kernels[updateGain4Id], 0, size_buffer, @ptrCast(&buffer_HPHTRI));
                try cl.kernel.set_arg(kernels[updateGain4Id], 1, size_buffer, @ptrCast(&buffer_HPHTRL));
                try cl.kernel.set_arg(kernels[updateGain4Id], 2, size_buffer, @ptrCast(&buffer_HPHTRU));
                try cl.kernel.set_arg(kernels[updateGain4Id], 3, @sizeOf(u32), &measure_dim);

                try cl.kernel.set_arg(kernels[updateGain5Id], 0, size_buffer, @ptrCast(&buffer_K));
                try cl.kernel.set_arg(kernels[updateGain5Id], 1, size_buffer, @ptrCast(&buffer_PHT));
                try cl.kernel.set_arg(kernels[updateGain5Id], 2, size_buffer, @ptrCast(&buffer_HPHTRI));
                try cl.kernel.set_arg(kernels[updateGain5Id], 3, @sizeOf(u32), &state_dim);
                try cl.kernel.set_arg(kernels[updateGain5Id], 4, @sizeOf(u32), &measure_dim);

                try cl.kernel.set_arg(kernels[updateStateId], 0, size_buffer, @ptrCast(&buffer_x));
                try cl.kernel.set_arg(kernels[updateStateId], 1, size_buffer, @ptrCast(&buffer_z));
                try cl.kernel.set_arg(kernels[updateStateId], 2, size_buffer, @ptrCast(&buffer_K));
                try cl.kernel.set_arg(kernels[updateStateId], 3, size_buffer, @ptrCast(&buffer_H));
                try cl.kernel.set_arg(kernels[updateStateId], 4, @sizeOf(u32), &state_dim);
                try cl.kernel.set_arg(kernels[updateStateId], 5, @sizeOf(u32), &measure_dim);

                try cl.kernel.set_arg(kernels[updateCovariance1Id], 0, size_buffer, @ptrCast(&buffer_IKH));
                try cl.kernel.set_arg(kernels[updateCovariance1Id], 1, size_buffer, @ptrCast(&buffer_K));
                try cl.kernel.set_arg(kernels[updateCovariance1Id], 2, size_buffer, @ptrCast(&buffer_H));
                try cl.kernel.set_arg(kernels[updateCovariance1Id], 3, @sizeOf(u32), &state_dim);
                try cl.kernel.set_arg(kernels[updateCovariance1Id], 4, @sizeOf(u32), &measure_dim);

                try cl.kernel.set_arg(kernels[updateCovariance2Id], 0, size_buffer, @ptrCast(&buffer_IKHP));
                try cl.kernel.set_arg(kernels[updateCovariance2Id], 1, size_buffer, @ptrCast(&buffer_IKH));
                try cl.kernel.set_arg(kernels[updateCovariance2Id], 2, size_buffer, @ptrCast(&buffer_P));
                try cl.kernel.set_arg(kernels[updateCovariance2Id], 3, @sizeOf(u32), &state_dim);

                try cl.kernel.set_arg(kernels[updateCovariance3Id], 0, size_buffer, @ptrCast(&buffer_IKHPIKHT));
                try cl.kernel.set_arg(kernels[updateCovariance3Id], 1, size_buffer, @ptrCast(&buffer_IKHP));
                try cl.kernel.set_arg(kernels[updateCovariance3Id], 2, size_buffer, @ptrCast(&buffer_IKH));
                try cl.kernel.set_arg(kernels[updateCovariance3Id], 3, @sizeOf(u32), &state_dim);

                try cl.kernel.set_arg(kernels[updateCovariance4Id], 0, size_buffer, @ptrCast(&buffer_KR));
                try cl.kernel.set_arg(kernels[updateCovariance4Id], 1, size_buffer, @ptrCast(&buffer_K));
                try cl.kernel.set_arg(kernels[updateCovariance4Id], 2, size_buffer, @ptrCast(&buffer_R));
                try cl.kernel.set_arg(kernels[updateCovariance4Id], 3, @sizeOf(u32), &state_dim);
                try cl.kernel.set_arg(kernels[updateCovariance4Id], 4, @sizeOf(u32), &measure_dim);

                try cl.kernel.set_arg(kernels[updateCovariance5Id], 0, size_buffer, @ptrCast(&buffer_P));
                try cl.kernel.set_arg(kernels[updateCovariance5Id], 1, size_buffer, @ptrCast(&buffer_IKHPIKHT));
                try cl.kernel.set_arg(kernels[updateCovariance5Id], 2, size_buffer, @ptrCast(&buffer_KR));
                try cl.kernel.set_arg(kernels[updateCovariance5Id], 3, size_buffer, @ptrCast(&buffer_K));
                try cl.kernel.set_arg(kernels[updateCovariance5Id], 4, @sizeOf(u32), &state_dim);
                try cl.kernel.set_arg(kernels[updateCovariance5Id], 5, @sizeOf(u32), &measure_dim);

                return OpenCLData{
                    .allocator = allocator,
                    .platform_id = platform_id,
                    .devices = devices,
                    .device_id = device_id,
                    .context = context,

                    .kernels = kernels,
                    .program = program,
                    .queue_state = queue_state,
                    .queue_covariance = queue_covariance,

                    .buffer_x = buffer_x,
                    .buffer_z = buffer_z,
                    .buffer_u = buffer_u,
                    .buffer_F = buffer_F,
                    .buffer_G = buffer_G,
                    .buffer_P = buffer_P,
                    .buffer_Q = buffer_Q,
                    .buffer_R = buffer_R,
                    .buffer_H = buffer_H,
                    .buffer_K = buffer_K,

                    .buffer_PHT = buffer_PHT,
                    .buffer_HPHTR = buffer_HPHTR,
                    .buffer_HPHTRL = buffer_HPHTRL,
                    .buffer_HPHTRU = buffer_HPHTRU,
                    .buffer_HPHTRI = buffer_HPHTRI,

                    .buffer_IKH = buffer_IKH,
                    .buffer_IKHP = buffer_IKHP,
                    .buffer_IKHPIKHT = buffer_IKHPIKHT,
                    .buffer_KR = buffer_KR,
                };
            }

            pub fn deinit(self: OpenCLData) void {
                for (self.kernels) |kernel| {
                    if (kernel) |k| cl.kernel.release(k) catch unreachable;
                }
                if (self.buffer_x) |x| cl.buffer.release(x) catch unreachable;
                if (self.buffer_z) |z| cl.buffer.release(z) catch unreachable;
                if (self.buffer_u) |u| cl.buffer.release(u) catch unreachable;
                if (self.buffer_F) |F| cl.buffer.release(F) catch unreachable;
                if (self.buffer_G) |G| cl.buffer.release(G) catch unreachable;
                if (self.buffer_P) |P| cl.buffer.release(P) catch unreachable;
                if (self.buffer_Q) |Q| cl.buffer.release(Q) catch unreachable;
                if (self.buffer_R) |R| cl.buffer.release(R) catch unreachable;
                if (self.buffer_H) |H| cl.buffer.release(H) catch unreachable;
                if (self.buffer_K) |K| cl.buffer.release(K) catch unreachable;

                if (self.buffer_PHT) |PHT| cl.buffer.release(PHT) catch unreachable;
                if (self.buffer_HPHTR) |HPHTR| cl.buffer.release(HPHTR) catch unreachable;
                if (self.buffer_HPHTRL) |HPHTRL| cl.buffer.release(HPHTRL) catch unreachable;
                if (self.buffer_HPHTRU) |HPHTRU| cl.buffer.release(HPHTRU) catch unreachable;
                if (self.buffer_HPHTRI) |HPHTRI| cl.buffer.release(HPHTRI) catch unreachable;

                if (self.buffer_IKH) |IKH| cl.buffer.release(IKH) catch unreachable;
                if (self.buffer_IKHP) |IKHP| cl.buffer.release(IKHP) catch unreachable;
                if (self.buffer_IKHPIKHT) |IKHPIKHT| cl.buffer.release(IKHPIKHT) catch unreachable;
                if (self.buffer_KR) |KR| cl.buffer.release(KR) catch unreachable;

                if (self.queue_covariance) |queue_covariance| cl.command_queue.release(queue_covariance) catch unreachable;
                if (self.queue_state) |queue_state| cl.command_queue.release(queue_state) catch unreachable;
                if (self.program) |program| cl.program.release(program) catch unreachable;
                if (self.context) |context| cl.context.release(context) catch unreachable;

                self.allocator.free(self.devices);
            }
        };
        aa: std.heap.ArenaAllocator,
        opencl_data: OpenCLData,
        log: *Log,

        pub fn init(
            config: Config,
            log: *Log,
            H: ?[measure_dim][state_dim]f32,
            R: ?[measure_dim][measure_dim]f32,
            x: ?[state_dim]f32,
            F: ?[state_dim][state_dim]f32,
            G: ?[state_dim][control_dim]f32,
            P: ?[state_dim][state_dim]f32,
            Q: ?[state_dim][state_dim]f32,
        ) !Self {
            var aa = std.heap.ArenaAllocator.init(std.heap.page_allocator);
            log.info("Initializing Kalman Filter", .{});

            const opencl_data = try OpenCLData.init(config.opencl, aa.allocator(), log);

            var kf = Self{
                .aa = aa,
                .opencl_data = opencl_data,
                .log = log,
            };

            const cl_data = &kf.opencl_data;
            const queue_state = cl_data.queue_state;
            if (H) |H_n| {
                try cl.buffer.write(queue_state, cl_data.buffer_H, true, 0, size_H, &H_n, null, null);
            }
            if (R) |R_n| {
                try cl.buffer.write(queue_state, cl_data.buffer_R, true, 0, size_R, &R_n, null, null);
            }

            _ = try kf.predict(
                x orelse [_]f32{0} ** state_dim,
                [_]f32{0} ** control_dim,
                F orelse [_][state_dim]f32{[_]f32{0} ** state_dim} ** state_dim,
                G orelse [_][control_dim]f32{[_]f32{0} ** control_dim} ** state_dim,
                P orelse [_][state_dim]f32{[_]f32{0} ** state_dim} ** state_dim,
                Q orelse [_][state_dim]f32{[_]f32{0} ** state_dim} ** state_dim,
            );

            return kf;
        }

        pub fn iterate(
            self: *Self,
            // update vars
            z: ?[measure_dim]f32,
            H: ?[measure_dim][state_dim]f32,
            R: ?[measure_dim][measure_dim]f32,
            // predict vars
            u: ?[control_dim]f32,
            F: ?[state_dim][state_dim]f32,
            G: ?[state_dim][control_dim]f32,
            Q: ?[state_dim][state_dim]f32,
        ) ![2][state_dim]f32 {
            // Estimate current state
            return [2][state_dim]f32{ try update(self, z, H, R), try predict(self, null, u, F, G, null, Q) };
        }

        // For time dependant vars, it should be how long it was since the last measurement
        fn predict(
            self: *Self,
            x: ?[state_dim]f32,
            u: ?[control_dim]f32,
            F: ?[state_dim][state_dim]f32,
            G: ?[state_dim][control_dim]f32,
            P: ?[state_dim][state_dim]f32,
            Q: ?[state_dim][state_dim]f32,
        ) ![state_dim]f32 {
            const log = self.log;
            const cl_data = &self.opencl_data;
            const queue_state = cl_data.queue_state;
            const queue_covariance = cl_data.queue_covariance;

            if (x) |x_n| try cl.buffer.write(queue_state, cl_data.buffer_x, true, 0, size_x, &x_n, null, null);
            if (u) |u_n| try cl.buffer.write(queue_state, cl_data.buffer_u, true, 0, size_u, &u_n, null, null);
            if (F) |F_n| try cl.buffer.write(queue_state, cl_data.buffer_F, true, 0, size_F, &F_n, null, null);
            if (G) |G_n| try cl.buffer.write(queue_state, cl_data.buffer_G, true, 0, size_G, &G_n, null, null);
            if (P) |P_n| try cl.buffer.write(queue_covariance, cl_data.buffer_P, true, 0, size_P, &P_n, null, null);
            if (Q) |Q_n| try cl.buffer.write(queue_covariance, cl_data.buffer_Q, true, 0, size_Q, &Q_n, null, null);

            var input_x: [state_dim]f32 = undefined;
            var input_u: [control_dim]f32 = undefined;
            var input_P: [state_dim][state_dim]f32 = undefined;
            var input_F: [state_dim][state_dim]f32 = undefined;
            var input_G: [state_dim][control_dim]f32 = undefined;
            var input_Q: [state_dim][state_dim]f32 = undefined;

            try cl.buffer.read(queue_state, cl_data.buffer_x, true, 0, size_x, &input_x, null, null);
            try cl.buffer.read(queue_state, cl_data.buffer_u, true, 0, size_u, &input_u, null, null);
            try cl.buffer.read(queue_covariance, cl_data.buffer_P, true, 0, size_P, &input_P, null, null);
            try cl.buffer.read(queue_covariance, cl_data.buffer_F, true, 0, size_F, &input_F, null, null);
            try cl.buffer.read(queue_covariance, cl_data.buffer_G, true, 0, size_G, &input_G, null, null);
            try cl.buffer.read(queue_covariance, cl_data.buffer_Q, true, 0, size_Q, &input_Q, null, null);

            log.info("Predict Called!", .{});
            log.debug("Starting Vars!", .{});
            log.debug("x: {d}", .{input_x});
            log.debug("u: {d}", .{input_u});
            log.debug("P:", .{});
            for (input_P) |p| {
                log.debug("{d:>7.2}", .{p});
            }
            log.debug("F:", .{});
            for (input_F) |f| {
                log.debug("{d:>7.2}", .{f});
            }
            log.debug("G:", .{});
            for (input_G) |g| {
                log.debug("{d:>7.2}", .{g});
            }
            log.debug("Q:", .{});
            for (input_Q) |q| {
                log.debug("{d:>10.8}", .{q});
            }

            // run kernels,
            var event_predict: cl.event.cl_event = undefined;
            try cl.kernel.enqueue_nd_range(
                queue_state,
                cl_data.kernels[@intFromEnum(Stage.PREDICT_STATE)],
                null,
                &[_]usize{state_dim},
                null,
                null,
                &event_predict,
            );
            try cl.kernel.enqueue_nd_range(
                queue_covariance,
                cl_data.kernels[@intFromEnum(Stage.PREDICT_COVARIANCE1)],
                null,
                &[_]usize{ state_dim, state_dim },
                null,
                null,
                &event_predict,
            );
            try cl.kernel.enqueue_nd_range(
                queue_covariance,
                cl_data.kernels[@intFromEnum(Stage.PREDICT_COVARIANCE2)],
                null,
                &[_]usize{ state_dim, state_dim },
                null,
                null,
                &event_predict,
            );

            try cl.event.wait(event_predict);
            try cl.event.release(event_predict);

            var new_x: [state_dim]f32 = undefined;
            var new_P: [state_dim][state_dim]f32 = undefined;
            try cl.buffer.read(queue_state, cl_data.buffer_x, true, 0, size_x, &new_x, null, null);
            try cl.buffer.read(queue_covariance, cl_data.buffer_P, true, 0, size_P, &new_P, null, null);

            log.info("Prediction Done.", .{});
            log.debug("x: {d}", .{new_x});
            log.debug("P:", .{});
            for (new_P) |p| {
                log.debug("{d:>7.2}", .{p});
            }
            return new_x;
        }

        // For time dependant vars, in update it should be how long it was since the last measurement
        fn update(
            self: *Self,
            z: ?[measure_dim]f32,
            H: ?[measure_dim][state_dim]f32,
            R: ?[measure_dim][measure_dim]f32,
        ) ![state_dim]f32 {
            const log = self.log;
            const cl_data = &self.opencl_data;
            const queue_state = cl_data.queue_state;
            const queue_covariance = cl_data.queue_covariance;

            if (z) |z_n| try cl.buffer.write(queue_state, cl_data.buffer_z, true, 0, size_z, &z_n, null, null);
            if (H) |H_n| try cl.buffer.write(queue_state, cl_data.buffer_H, true, 0, size_H, &H_n, null, null);
            if (R) |R_n| try cl.buffer.write(queue_state, cl_data.buffer_R, true, 0, size_R, &R_n, null, null);

            var input_z: [measure_dim]f32 = undefined;
            var input_H: [measure_dim][state_dim]f32 = undefined;
            var input_R: [measure_dim][measure_dim]f32 = undefined;

            try cl.buffer.read(queue_state, cl_data.buffer_z, true, 0, size_z, &input_z, null, null);
            try cl.buffer.read(queue_state, cl_data.buffer_H, true, 0, size_H, &input_H, null, null);
            try cl.buffer.read(queue_state, cl_data.buffer_R, true, 0, size_R, &input_R, null, null);

            log.info("Update Called!", .{});
            log.debug("Starting Vars!", .{});
            log.debug("z: {d}", .{input_z});
            log.debug("H: {d}", .{input_H});
            log.debug("R:", .{});
            for (input_R) |r| {
                log.debug("{d:>7.2}", .{r});
            }

            // run kernels,
            try cl.kernel.enqueue_nd_range(
                queue_covariance,
                cl_data.kernels[@intFromEnum(Stage.UPDATE_GAIN1)],
                null,
                &[_]usize{ state_dim, measure_dim },
                null,
                null,
                null,
            );
            try cl.kernel.enqueue_nd_range(
                queue_covariance,
                cl_data.kernels[@intFromEnum(Stage.UPDATE_GAIN2)],
                null,
                &[_]usize{ measure_dim, measure_dim },
                null,
                null,
                null,
            );
            try cl.kernel.enqueue_nd_range(
                queue_covariance,
                cl_data.kernels[@intFromEnum(Stage.UPDATE_GAIN3)],
                null,
                &[_]usize{measure_dim},
                null,
                null,
                null,
            );
            try cl.kernel.enqueue_nd_range(
                queue_covariance,
                cl_data.kernels[@intFromEnum(Stage.UPDATE_GAIN4)],
                null,
                &[_]usize{measure_dim},
                null,
                null,
                null,
            );
            try cl.kernel.enqueue_nd_range(
                queue_covariance,
                cl_data.kernels[@intFromEnum(Stage.UPDATE_GAIN5)],
                null,
                &[_]usize{ state_dim, measure_dim },
                null,
                null,
                null,
            );

            try cl.command_queue.finish(queue_covariance);
            var event_update: cl.event.cl_event = undefined;

            try cl.kernel.enqueue_nd_range(
                queue_state,
                cl_data.kernels[@intFromEnum(Stage.UPDATE_STATE)],
                null,
                &[_]usize{@max(state_dim, measure_dim)},
                null,
                null,
                &event_update,
            );
            try cl.kernel.enqueue_nd_range(
                queue_covariance,
                cl_data.kernels[@intFromEnum(Stage.UPDATE_COVARIANCE1)],
                null,
                &[_]usize{ state_dim, state_dim },
                null,
                null,
                &event_update,
            );
            try cl.kernel.enqueue_nd_range(
                queue_covariance,
                cl_data.kernels[@intFromEnum(Stage.UPDATE_COVARIANCE2)],
                null,
                &[_]usize{ state_dim, state_dim },
                null,
                null,
                &event_update,
            );
            try cl.kernel.enqueue_nd_range(
                queue_covariance,
                cl_data.kernels[@intFromEnum(Stage.UPDATE_COVARIANCE3)],
                null,
                &[_]usize{ state_dim, state_dim },
                null,
                null,
                &event_update,
            );
            try cl.kernel.enqueue_nd_range(
                queue_covariance,
                cl_data.kernels[@intFromEnum(Stage.UPDATE_COVARIANCE4)],
                null,
                &[_]usize{ state_dim, measure_dim },
                null,
                null,
                &event_update,
            );
            try cl.kernel.enqueue_nd_range(
                queue_covariance,
                cl_data.kernels[@intFromEnum(Stage.UPDATE_COVARIANCE5)],
                null,
                &[_]usize{ state_dim, state_dim },
                null,
                null,
                &event_update,
            );

            try cl.event.wait(event_update);
            try cl.event.release(event_update);

            var new_x: [state_dim]f32 = undefined;
            var new_P: [state_dim][state_dim]f32 = undefined;
            var new_K: [state_dim]f32 = undefined;
            try cl.buffer.read(queue_state, cl_data.buffer_x, true, 0, size_x, &new_x, null, null);
            try cl.buffer.read(queue_covariance, cl_data.buffer_P, true, 0, size_P, &new_P, null, null);
            try cl.buffer.read(queue_covariance, cl_data.buffer_K, true, 0, size_K, &new_K, null, null);

            log.info("Update Done.", .{});
            log.debug("x: {d}", .{new_x});
            log.debug("P:", .{});
            for (new_P) |p| {
                log.debug("{d:>7.2}", .{p});
            }
            log.debug("K:", .{});
            for (new_K) |k| {
                log.debug("{d:>5.4}", .{k});
            }
            return new_x;
        }

        pub fn deinit(self: *Self) void {
            self.log.info("Deinitializing", .{});
            self.opencl_data.deinit();

            self.aa.deinit();
        }
    };
}

test "Example" {
    var log = try Log.init(.{
        .level = .DEBUG,
        .abs_path = "/tmp/CLAW/test/KalmanFilter",
        .channel = "test",
    });
    const KF = KalmanFilter(2, 1, 1);
    const config = KF.Config{
        .opencl = .{
            .platform_name = "NVIDIA CUDA",
            .device_name = "NVIDIA GeForce RTX 3080",
        },
    };

    // Example from "Kalman Filter from the Ground Up, Second Edition" Chapter 9.2
    var kf = try KF.init(
        config,
        &log,
        [1][2]f32{.{ 1, 0 }}, // H
        [1][1]f32{.{400}}, // R
        null, // x
        [2][2]f32{ .{ 1, 0.25 }, .{ 0, 1 } }, // F
        [2][1]f32{ .{0.0313}, .{0.25} }, // G
        [2][2]f32{ .{ 500, 0 }, .{ 0, 500 } }, // P
        [2][2]f32{ .{ 9.765625e-6, 7.8125e-3 }, .{ 7.8125e-3, 6.25e-2 } }, // Q
    );
    defer kf.deinit();

    const x = try kf.iterate(
        [1]f32{6.43}, // z
        null, // H
        null, // R
        [1]f32{39.81 - 9.81}, // u (velocity minus the effect of gravity)
        null, // F
        null, // G
        null, // Q
    );

    log.info("x_1: {d}", .{x});
    try std.testing.expectApproxEqAbs(x[0][0], 3.67, 0.01);
    try std.testing.expectApproxEqAbs(x[0][1], 0.86, 0.01);
    try std.testing.expectApproxEqAbs(x[1][0], 4.82, 0.01);
    try std.testing.expectApproxEqAbs(x[1][1], 8.36, 0.01);
}
