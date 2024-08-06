// Kalman filter Kernels

// x: estimated state vector        n_x * 1
// z: measurements vector           n_z * 1
// u: input vector                  n_u * 1
// F: state transition matrix       n_x * n_x
// G: control matrix                n_x * n_u
// P: estimate covariance matrix    n_x * n_x
// Q: process noise covariance      n_x * n_x
// R: measurement noise covariance  n_z * n_z
// H: observation matrix            n_z * n_x
// K: Kalman gain matrix            n_x * n_z

// x_n+1,n = F * x_n,n + G * u_n
__kernel void PREDICT_STATE(
    __global float* x,
    __global const float* u,
    __global const float* F,
	__global const float* G,
    int state_dim,
	int control_dim
    )
{
    int row = get_global_id(0);
    if (row < state_dim) {
        float sum = 0.0f;
        for (int col = 0; col < state_dim; col++) {
            sum += F[row * state_dim + col] * x[col];
        }
        x[row] = sum;
    }

    if (row < state_dim) {
        float sum = 0.0f;
        for (int col = 0; col < control_dim; col++) {
            sum += G[row * control_dim + col] * u[col];
        }
        x[row] += sum;
    }
}

// P_n+1,n = F * P_n,n * F^T + Q
__kernel void PREDICT_COVARIANCE1(
    __global float* P,
    __global float* F,
    int state_dim
    )
{
    int row = get_global_id(0);
    int col = get_global_id(1);

    // F * P_n,n
    if (row < state_dim && col < state_dim) {
        float sum = 0.0f;
        for (int k = 0; k < state_dim; k++) {
            sum += F[row * state_dim + k] * P[k * state_dim + col];
        }
        P[row * state_dim + col] = sum;
    }
}
__kernel void PREDICT_COVARIANCE2(
    __global float* P,
    __global const float* F,
    __global const float* Q,
    int state_dim
    )
{
    int row = get_global_id(0);
    int col = get_global_id(1);
    // F * P_n,n * F^T
    if (row < state_dim && col < state_dim) {
        float sum = 0.0f;
        for (int k = 0; k < state_dim; k++) {
            sum += P[row * state_dim + k] * F[col * state_dim + k];
        }
        P[row * state_dim + col] = sum + Q[row * state_dim + col];
    }
}

// K_n = P_n,n-1 * H^T * (H * P_n,n-1 * H^T + R_n)^-1
__kernel void UPDATE_GAIN1(
    __global float* PHT,
    __global const float* P,
    __global const float* H,
    int state_dim,
	int measure_dim
    )
{
    int row = get_global_id(0);
    int col = get_global_id(1);

    // P_n,n-1 * H^T
    if (row < state_dim && col < measure_dim) {
        float sum = 0.0f;
        for (int k = 0; k < state_dim; k++) {
            sum += P[row * state_dim + k] * H[col * state_dim + k];
        }
        PHT[row * measure_dim + col] = sum;
    }
}
__kernel void UPDATE_GAIN2(
    __global float* HPHTR,
    __global const float* PHT,
    __global const float* H,
    __global const float* R,
    int state_dim,
	int measure_dim
    )
{
	// (H * P_n,n-1 * H^T + R_n)
	int row = get_global_id(0);
	int col = get_global_id(1);

	if (row < measure_dim && col < measure_dim) {
		float sum = 0.0f;
		for (int k = 0; k < state_dim; k++) {
			sum += H[row * state_dim + k] * PHT[k * measure_dim + col];
		}
		HPHTR[row * measure_dim + col] = sum + R[row * measure_dim + col];
	}
}
__kernel void UPDATE_GAIN3(
    __global float* HPHTR_L,
    __global float* HPHTR_U,
    __global const float* HPHTR,
    int measure_dim
    )
{
	// (H * P_n,n-1 * H^T + R_n)^-1
	// Pt 1. LU decomposition
    int i, j, k;
    int gid = get_global_id(0);

    // Initialize L and U
    for (i = gid; i < measure_dim; i += get_global_size(0)) {
        for (j = 0; j < measure_dim; j++) {
            if (i == j) {
                HPHTR_L[i * measure_dim + i] = 1.0;
            } else if (i > j) {
                HPHTR_L[i * measure_dim + j] = 0.0;
                HPHTR_U[i * measure_dim + j] = 0.0;
            } else {
                HPHTR_L[i * measure_dim + j] = 0.0;
                HPHTR_U[i * measure_dim + j] = HPHTR[i * measure_dim + j];
            }
        }
    }

    barrier(CLK_GLOBAL_MEM_FENCE);

    // LU Decomposition
    for (k = 0; k < measure_dim; k++) {
        for (i = gid; i < measure_dim; i += get_global_size(0)) {
            if (i >= k) {
                HPHTR_U[k * measure_dim + i] = HPHTR[k * measure_dim + i];
                for (j = 0; j < k; j++) {
                    HPHTR_U[k * measure_dim + i] -= HPHTR_L[k * measure_dim + j] * HPHTR_U[j * measure_dim + i];
                }
            }

            if (i >= k + 1) {
                HPHTR_L[i * measure_dim + k] = HPHTR[i * measure_dim + k] / HPHTR_U[k * measure_dim + k];
                for (j = 0; j < k; j++) {
                    HPHTR_L[i * measure_dim + k] -= (HPHTR_L[i * measure_dim + j] * HPHTR_U[j * measure_dim + k]) / HPHTR_U[k * measure_dim + k];
                }
            }
        }

        barrier(CLK_GLOBAL_MEM_FENCE);
    }
}

__kernel void UPDATE_GAIN4(
    __global float* HPHTRI,
    __global const float* HPHTR_L,
    __global const float* HPHTR_U,
    int measure_dim
    )
{
	// (H * P_n,n-1 * H^T + R_n)^-1
	// Pt 2. Backward and forward substitution
    int i, j, k;
    int gid = get_global_id(0);

	// TODO: make zig set local size based on measure_dim and state_dim at compile time
    __local float y[1024];
    __local float x[1024];

    for (int col = 0; col < measure_dim; ++col) {
        // Forward substitution to solve Ly = b
        for (i = gid; i < measure_dim; i += get_global_size(0)) {
            if (i == col) {
                y[i] = 1.0;
            } else {
                y[i] = 0.0;
            }

            for (j = 0; j < i; j++) {
                y[i] -= HPHTR_L[i * measure_dim + j] * y[j];
            }
        }

        barrier(CLK_GLOBAL_MEM_FENCE|CLK_LOCAL_MEM_FENCE);

        // Backward substitution to solve Ux = y
        for (i = measure_dim - 1 - gid; i >= 0; i -= get_global_size(0)) {
            x[i] = y[i];

            for (j = i + 1; j < measure_dim; j++) {
                x[i] -= HPHTR_U[i * measure_dim + j] * x[j];
            }

            x[i] /= HPHTR_U[i * measure_dim + i];
        }

        barrier(CLK_GLOBAL_MEM_FENCE|CLK_LOCAL_MEM_FENCE);

        // Write result to the corresponding column of the inverse matrix
        for (i = gid; i < measure_dim; i += get_global_size(0)) {
			HPHTRI[i * measure_dim + col] = x[i];
        }

        barrier(CLK_GLOBAL_MEM_FENCE|CLK_LOCAL_MEM_FENCE);
	}
}

__kernel void UPDATE_GAIN5(
	__global float* K,
    __global const float* PHT,
    __global const float* HPHTRI,
    int state_dim,
    int measure_dim
    )
{
	// K_n = P_n,n-1 * H^T * (H * P_n,n-1 * H^T + R_n)^-1
    int row = get_global_id(0);
    int col = get_global_id(1);

    // Check bounds
    if (row < state_dim && col < measure_dim) {
        float sum = 0.0f;
        for (int k = 0; k < measure_dim; ++k) {
            sum += PHT[row * measure_dim + k] * HPHTRI[k * measure_dim + col];
        }
        K[row * measure_dim + col] = sum;
    }
}

// x_n,n = x_n,n-1 + K_n * (z_n - H * x_n,n-1)
__kernel void UPDATE_STATE(
    __global float* x,
    __global const float* z,
    __global const float* K,
    __global const float* H,
    const int s,
    const int m
) {
	// TODO: make zig set local size based on measure_dim and state_dim at compile time
    __local float Hx[1024];

    // Get the global ID of the current work item
    int gid = get_global_id(0);

    // Compute H * x and store it in local memory
    if (gid < m) {
        float sum = 0.0f;
        for (int j = 0; j < s; ++j) {
            sum += H[gid * s + j] * x[j];
        }
        Hx[gid] = sum;
    }

    barrier(CLK_LOCAL_MEM_FENCE);

    // Compute z - H * x and then K * (z - H * x) and update x
    if (gid < s) {
        float sum = 0.0f;
        for (int j = 0; j < m; ++j) {
            sum += K[gid * m + j] * (z[j] - Hx[j]);
        }
        x[gid] += sum;
    }
}

__kernel void UPDATE_COVARIANCE1(
	__global float* IKH,
	__global const float* K,
	__global const float* H,
    const int s,
    const int m
	)
{
    //(I - K_n * H)
    int row = get_global_id(0);
    int col = get_global_id(1);

    if (row < s && col < s) {
        float sum = 0.0f;

        // Compute the product K * H
        for (int k = 0; k < m; ++k) {
            sum += K[row * m + k] * H[k * s + col];
        }

        // Compute I - K * H
        if (row == col) {
            IKH[row * s + col] = 1.0f - sum;
        } else {
            IKH[row * s + col] = -sum;
        }
    }
}

__kernel void UPDATE_COVARIANCE2(
    __global float* IKHP,
    __global const float* IKH,
    __global const float* P,
    const int s
) {
    // (I - K_n * H) * P_n,n-1
    int row = get_global_id(0);
    int col = get_global_id(1);

    if (row < s && col < s) {
        float sum = 0.0f;

        // Compute the result[row, col] element
        for (int k = 0; k < s; ++k) {
            sum += IKH[row * s + k] * P[k * s + col];
        }
		IKHP[row * s + col] = sum;
    }
}


__kernel void UPDATE_COVARIANCE3(
    __global float* IKHPIKHT,
    __global const float* IKHP,
    __global const float* IKH,
    const int s
) {
    // (I - K_n * H) * P_n,n-1 * (I - K_n * H)^T
    int row = get_global_id(0);
    int col = get_global_id(1);

    if (row < s && col < s) {
        float sum = 0.0f;

        // Compute the result[row, col] element
        for (int k = 0; k < s; ++k) {
            sum += IKHP[row * s + k] * IKH[col * s + k];
        }
		IKHPIKHT[row * s + col] = sum;
    }
}

__kernel void UPDATE_COVARIANCE4(
    __global float* KR,
    __global const float* K,
    __global const float* R,
    const int s,
    const int m
) {
	// K_n * R_n
	int row = get_global_id(0);
	int col = get_global_id(1);

	if (row < s && col < m) {
		float sum = 0.0f;

		for (int k = 0; k < m; ++k) {
			sum += K[row * m + k] * R[k * m + col];
		}
		KR[row * m + col] = sum;
	}
}

__kernel void UPDATE_COVARIANCE5(
	__global float* P,
	__global const float* IKHPIKHT,
	__global const float* KR,
	__global const float* K,
	const int s,
	const int m
) {
	// P_n,n = (I - K_n * H) * P_n,n-1 * (I - K_n * H)^T + K_n * R_n * K_n^T
	int row = get_global_id(0);
	int col = get_global_id(1);

	if (row < s && col < s) {
		float sum = 0.0f;

		for (int k = 0; k < m; ++k) {
			sum += KR[row * m + k] * K[col * m + k];
		}
		P[row * s + col] = IKHPIKHT[row * s + col] + sum;
		//P[row * s + col] = sum;
	}
}
