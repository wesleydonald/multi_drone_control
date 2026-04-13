import numpy as np
from scipy.linalg import cholesky
import time


class UKFEstimator:
    """
    Unscented Kalman Filter for adaptive parameter estimation in omnicopter control.
    
    Estimates the 5 adaptive parameters: [theta_roll, theta_pitch, theta_yaw, 
    thrust_base, motor_time_constant] using only pose observations.
    The 18-dimensional state vector (13D pose + 5D parameters).
    Mass, motor_distance, and inertia are treated as known constants.
    """
    
    def __init__(self, sim_integrator, dt=1.0/30.0, initial_params=None):
        """
        Initialize UKF estimator.
        
        Args:
            sim_integrator: Acados simulation solver for state propagation
            dt: Control loop time step in seconds
            initial_params: Initial adaptive parameters [theta_roll, theta_pitch, theta_yaw, thrust_base, motor_time_constant]
        """
        self.sim_integrator = sim_integrator
        self.dt = dt
        
        # UKF tuning parameters - balanced for numerical stability
        self.alpha = 0.1   # Moderate spread for good sigma point diversity
        self.beta = 2.0    # Incorporate prior knowledge of distribution (2 = optimal for Gaussian)
        self.kappa = 1.0   # Positive kappa for better conditioning
        
        # State dimensions - simplified to only track pose + parameters
        self.n_pose = 13      # [px, py, pz, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz]
        self.n_params = 5     # [theta_roll, theta_pitch, theta_yaw, thrust_base, motor_time_constant]
        self.n_state = self.n_pose + self.n_params  # 18 total
        self.n_obs = self.n_pose  # Observable states from motion capture
        
        # Initialize state estimate (pose + parameters only)
        self.x_est = np.zeros(self.n_state)
        self.x_est[3] = 1.0  # Initialize quaternion w component
        
        # Initialize adaptive parameters to provided values or nominal defaults
        if initial_params is not None:
            self.x_est[13:18] = initial_params.copy()
        else:
            self.x_est[13:18] = np.array([0.0, 0.0, 0.0, 7.42678162, 0.12])
        
        # Initialize covariance matrix
        self.P = self._initialize_covariance()
        
        # Process noise covariance Q
        self.Q = self._initialize_process_noise()
        
        # Measurement noise covariance R
        self.R = self._initialize_measurement_noise()
        
        # Control history for delay compensation
        self.control_history = []
        self.delay_states = 1  # Number of time steps of delay
        
        # Storage for estimated parameters
        self.est_params = self.x_est[13:18].copy()
        
        # Store current actuator states for the dynamics simulation
        # These are managed externally and used only for simulation
        self.current_actuator_states = np.array([-0.20, 0.20, -0.20, 0.20, 0.20, -0.20, 0.20, -0.20])
        self.current_desired_states = np.array([-0.20, 0.20, -0.20, 0.20, 0.20, -0.20, 0.20, -0.20])
        
        # Step counter for periodic reporting
        self.step_counter = 0
        
    def _initialize_covariance(self):
        """Initialize state covariance matrix with appropriate uncertainties."""
        P = np.eye(self.n_state)
        
        # Position uncertainty (m)
        P[0:3, 0:3] *= 0.01
        
        # Quaternion uncertainty (small for unit quaternion)
        P[3:7, 3:7] *= 0.001
        
        # Velocity uncertainty (m/s)
        P[7:10, 7:10] *= 0.05
        
        # Angular velocity uncertainty (rad/s)
        P[10:13, 10:13] *= 0.05
        
        # Parameter uncertainties - balanced for stable adaptation
        P[13, 13] = 1e-6      # theta_roll uncertainty (increased for stability)
        P[14, 14] = 1e-6      # theta_pitch uncertainty (increased for stability)
        P[15, 15] = 1e-6      # theta_yaw uncertainty (increased for stability)
        P[16, 16] = 1e-2       # thrust_base uncertainty (around 7.42) - reasonable for adaptation
        P[17, 17] = 1e-3      # motor_time_constant uncertainty (s)
        
        return P
        
    def _initialize_process_noise(self):
        """Initialize process noise covariance matrix."""
        Q = np.eye(self.n_state)
        
        # Pose states - small process noise
        Q[0:13, 0:13] *= 1e-6
        
        # Parameter process noise (conservative but not too aggressive)
        Q[13, 13] = 1e-8     # theta_roll (slow adaptation)
        Q[14, 14] = 1e-8     # theta_pitch (slow adaptation)
        Q[15, 15] = 1e-8     # theta_yaw (slow adaptation)
        Q[16, 16] = 0.01     # thrust_base (moderate adaptation for values around 7.42)
        Q[17, 17] = 1e-8     # motor_time_constant (slow adaptation)
        
        return Q
        
    def _initialize_measurement_noise(self):
        """Initialize measurement noise covariance matrix."""
        R = np.eye(self.n_obs)
        
        # Motion capture position noise (m)
        R[0:3, 0:3] *= 0.001
        
        # Motion capture quaternion noise
        R[3:7, 3:7] *= 0.001
        
        # Motion capture velocity noise (m/s)
        R[7:10, 7:10] *= 0.05
        
        # Motion capture angular velocity noise (rad/s)
        R[10:13, 10:13] *= 0.05
        
        return R
        
    def generate_sigma_points(self, x, P, alpha, beta, kappa):
        """
        Generate sigma points for the unscented transform.
        
        Args:
            x: State vector
            P: Covariance matrix
            alpha, beta, kappa: UKF tuning parameters
            
        Returns:
            sigma_points: Array of sigma points
            weights_mean: Weights for mean calculation
            weights_cov: Weights for covariance calculation
        """
        n = len(x)
        lambda_ = alpha**2 * (n + kappa) - n
        sigma_points = [x]
        
        # Add numerical stability to the covariance matrix
        P_stable = P + np.eye(n) * 1e-6  # Moderate regularization
        
        # Check if matrix is positive definite
        try:
            sqrt_P = cholesky((n + lambda_) * P_stable, lower=True)
        except np.linalg.LinAlgError:
            # Use eigenvalue decomposition for more stability
            eigenvals, eigenvecs = np.linalg.eigh((n + lambda_) * P_stable)
            eigenvals = np.maximum(eigenvals, 1e-8)  # Ensure positive eigenvalues
            sqrt_P = eigenvecs @ np.diag(np.sqrt(eigenvals))
        
        for i in range(n):
            sigma_points.append(x + sqrt_P[:, i])
            sigma_points.append(x - sqrt_P[:, i])
            
        weights_mean = [lambda_ / (n + lambda_)] + [1 / (2 * (n + lambda_))] * 2 * n
        weights_cov = [lambda_ / (n + lambda_) + (1 - alpha**2 + beta)] + [1 / (2 * (n + lambda_))] * 2 * n
        
        return np.array(sigma_points), np.array(weights_mean), np.array(weights_cov)
        
    def unscented_transform(self, sigma_points, weights_mean, weights_cov, noise_cov=None):
        """
        Perform unscented transform on sigma points.
        
        Args:
            sigma_points: Array of transformed sigma points
            weights_mean: Weights for mean calculation
            weights_cov: Weights for covariance calculation
            noise_cov: Additional noise covariance to add
            
        Returns:
            mean: Transformed mean
            cov: Transformed covariance
        """
        mean = np.sum(weights_mean[:, None] * sigma_points, axis=0)
        cov = np.zeros((mean.size, mean.size))
        
        for i in range(sigma_points.shape[0]):
            dx = sigma_points[i] - mean
            cov += weights_cov[i] * np.outer(dx, dx)
            
        if noise_cov is not None:
            cov += noise_cov
            
        return mean, cov
        
    def fx(self, x, u_dot_rates):
        """
        Process model: propagate state forward one time step.
        
        Since we only track pose + parameters (18D), we need to use external
        actuator states for the dynamics simulation.
        
        Args:
            x: Current state [pose(13) + parameters(5)] = 18D
            u_dot_rates: Control input (actuator rate commands)
            
        Returns:
            x_next: Predicted next state [pose_next(13) + parameters(5)]
        """
        # Extract components from reduced state
        pose = x[:13]           # pose(13)
        params = x[13:18]       # adaptive parameters(5)
        
        # Create full 29D state for simulation using current actuator estimates
        # This is only for simulation - we don't track these in the UKF state
        full_state = np.concatenate([
            pose,                              # pose (13D)
            self.current_actuator_states,      # actual actuators (8D)
            self.current_desired_states        # desired actuators (8D)
        ])
        
        # Set the state, input, and parameters in the simulation integrator
        self.sim_integrator.set("x", full_state)
        self.sim_integrator.set("u", u_dot_rates)
        self.sim_integrator.set("p", params)
        
        # Perform the integration step
        status = self.sim_integrator.solve()
        if status != 0:
            print(f"Warning: UKF simulation integrator returned status {status}")
            
        # Retrieve the next state from the integrator
        next_full_state = self.sim_integrator.get("x")
        next_pose = next_full_state[:13]  # Extract only the pose portion
        
        # Update our internal actuator state estimates for next iteration
        self.current_actuator_states = next_full_state[13:21].copy()
        self.current_desired_states = next_full_state[21:29].copy()
        
        # Parameters remain constant during prediction (process noise will add variation)
        x_next = np.concatenate([next_pose, params])
        
        return x_next
        
    def hx(self, x):
        """
        Measurement model: extract observable states.
        
        Args:
            x: Full state vector
            
        Returns:
            z: Observable measurements (pose only from motion capture)
        """
        return x[:13]  # Return only pose states (position, quaternion, velocity, angular velocity)
        
    def predict_and_update(self, measurement, u_dot_rates):
        """
        Perform one complete UKF predict and update cycle.
        
        Args:
            measurement: Current measurement from motion capture [pose(13)]
            u_dot_rates: Control input applied
            
        Returns:
            estimated_params: Updated parameter estimates
        """
        start_time = time.time()
        
        # UKF Predict Step
        sigma_pts, wm, wc = self.generate_sigma_points(self.x_est, self.P, self.alpha, self.beta, self.kappa)
        
        # Use delayed control input if available
        if len(self.control_history) >= self.delay_states:
            delayed_u = np.array(self.control_history[-self.delay_states])
        else:
            delayed_u = u_dot_rates
            
        sigma_pts_pred = np.array([self.fx(pt, delayed_u) for pt in sigma_pts])
        x_pred, P_pred = self.unscented_transform(sigma_pts_pred, wm, wc, self.Q)
        
        # UKF Update Step
        sigma_meas = np.array([self.hx(pt) for pt in sigma_pts_pred])
        z_pred, P_zz = self.unscented_transform(sigma_meas, wm, wc, self.R)
        
        # Calculate cross-covariance
        P_xz = np.zeros((x_pred.size, z_pred.size))
        for i in range(sigma_pts.shape[0]):
            dx = sigma_pts_pred[i] - x_pred
            dz = sigma_meas[i] - z_pred
            P_xz += wc[i] * np.outer(dx, dz)
            
        # Kalman gain and state update
        K = P_xz @ np.linalg.inv(P_zz)
        
        # Apply standard Kalman gain
        innovation = measurement - z_pred
        self.x_est = x_pred + K @ innovation
        self.P = P_pred - K @ P_zz @ K.T
        
        # Ensure P remains positive definite with improved conditioning
        self.P = 0.5 * (self.P + self.P.T)  # Make symmetric
        
        # Check condition number and eigenvalues
        eigenvals = np.linalg.eigvals(self.P)
        min_eigval = np.min(eigenvals)
        condition_number = np.max(eigenvals) / max(min_eigval, 1e-12)
        
        # Apply regularization based on condition number or negative eigenvalues
        if min_eigval < 1e-8 or condition_number > 1e8:
            if min_eigval < 1e-8:
                print(f"Warning: Covariance matrix has small eigenvalue {min_eigval:.2e}, adding regularization")
            else:
                print(f"Warning: Covariance matrix poorly conditioned (cond={condition_number:.2e}), adding regularization")
            
            # Adaptive regularization based on severity
            reg_strength = max(1e-6, -2 * min_eigval) if min_eigval < 0 else 1e-6
            self.P += np.eye(self.P.shape[0]) * reg_strength
            
        # Normalize quaternion to ensure it remains a valid unit quaternion
        quat_norm = np.linalg.norm(self.x_est[3:7])
        if quat_norm > 0:
            self.x_est[3:7] = self.x_est[3:7] / quat_norm
            
        # Constrain parameters to physically reasonable bounds with rate limiting
        old_params = self.x_est[13:18].copy()
        
        # Apply physical bounds
        self.x_est[13] = np.clip(self.x_est[13], -0.5, 0.5)      # theta_roll (rad)
        self.x_est[14] = np.clip(self.x_est[14], -0.5, 0.5)      # theta_pitch (rad)
        self.x_est[15] = np.clip(self.x_est[15], -0.5, 0.5)      # theta_yaw (rad)
        self.x_est[16] = np.clip(self.x_est[16], 4.0, 20.0)           # thrust_base (around 7.42)
        self.x_est[17] = np.clip(self.x_est[17], 0.04, 0.2)           # motor_time_constant (s)
        
        # Apply rate limiting to prevent sudden parameter jumps
        max_param_change_rates = np.array([0.01, 0.01, 0.01, 0.1, 0.001])  # per time step
        param_changes = self.x_est[13:18] - old_params
        param_change_magnitudes = np.abs(param_changes)
        
        # Limit changes that are too large
        for i, (change_mag, max_rate) in enumerate(zip(param_change_magnitudes, max_param_change_rates)):
            if change_mag > max_rate:
                # Scale down the change
                scale_factor = max_rate / change_mag
                self.x_est[13 + i] = old_params[i] + param_changes[i] * scale_factor
        
        # Update estimated parameters
        old_est_params = self.est_params.copy()
        self.est_params = self.x_est[13:18].copy()
        
        # Debug parameter changes
        param_changes = self.est_params - old_est_params
        if np.any(np.abs(param_changes) > 1e-6):
            param_names = ['theta_roll', 'theta_pitch', 'theta_yaw', 'thrust_base', 'motor_time_constant']
            print("UKF Parameter Changes:")
            for i, (name, change) in enumerate(zip(param_names, param_changes)):
                if abs(change) > 1e-6:
                    print(f"  {name}: {change:.8f} (new: {self.est_params[i]:.6f})")
        
        self.step_counter += 1
        # Update control history
        self.control_history.append(u_dot_rates.tolist())
        if len(self.control_history) > 100:  # Keep history reasonable
            self.control_history.pop(0)
            
        # Calculate execution time and monitor numerical health
        end_time = time.time()
        execution_time = (end_time - start_time) * 1000
        
        # Monitor covariance matrix health
        P_condition = np.linalg.cond(self.P)
        if P_condition > 1e6:
            print(f"Warning: Covariance matrix condition number high: {P_condition:.2e}")
        
        if execution_time > 10.0:  # Warn if UKF takes too long
            print(f"Warning: UKF execution time: {execution_time:.2f} ms")
            
        return self.est_params
        
    def get_estimated_state(self):
        """Get the current pose state estimate."""
        return self.x_est[:13]
        
    def get_estimated_parameters(self):
        """Get the current parameter estimates."""
        return self.est_params
        
    def set_initial_state(self, initial_pose, initial_actuators=None):
        """
        Set initial state estimate.
        
        Args:
            initial_pose: Initial pose estimate [13D]
            initial_actuators: Initial actuator states [16D] - used to initialize internal actuator tracking
        """
        self.x_est[:13] = initial_pose
        
        # If actuator states are provided, use them to initialize our internal tracking
        if initial_actuators is not None:
            self.current_actuator_states = initial_actuators[:8].copy()   # Actual actuator states
            self.current_desired_states = initial_actuators[8:16].copy()  # Desired actuator states
        else:
            # Use default hover values
            hover_values = np.array([-0.28, 0.28, -0.28, 0.28, 0.28, -0.28, 0.28, -0.28])
            self.current_actuator_states = hover_values.copy()
            self.current_desired_states = hover_values.copy()
            
        # Normalize quaternion
        quat_norm = np.linalg.norm(self.x_est[3:7])
        if quat_norm > 0:
            self.x_est[3:7] = self.x_est[3:7] / quat_norm
    
    def update_actuator_states(self, actual_actuators, desired_actuators):
        """
        Update the internal actuator state tracking.
        
        Args:
            actual_actuators: Current actual actuator states [8D]
            desired_actuators: Current desired actuator states [8D]
        """
        self.current_actuator_states = np.array(actual_actuators).copy()
        self.current_desired_states = np.array(desired_actuators).copy()