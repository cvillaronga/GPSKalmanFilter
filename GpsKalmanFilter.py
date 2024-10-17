import numpy as np
from dataclasses import dataclass

@dataclass
class Location:
    latitude: float
    longitude: float
    altitude: float
    accuracy: float = 0.0

@dataclass
class Point:
    _x: float
    _y: float
    _z: float
    
    def x(self):
        return self._x
        
    def y(self):
        return self._y
        
    def z(self):
        return self._z
    
    @staticmethod
    def geocentric_to_geodetic(x, y, z):
        # Note: This is a placeholder - you'll need to implement the actual conversion
        return Location(x, y, z)

class GpsKalmanFilter:
    def __init__(self, error):
        # Noise in the world
        self.pos = 0.00001
        
        # discrete time interval
        self.dt = 0.1
        
        # Calculate error scaling
        self.xv = (3 ** -0.5) * error
        
        # Initialize matrices
        # (A) State transition matrix
        self.A = np.array([
            [1.0, self.xv, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, self.xv, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, self.xv],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        ])
        
        # (B) Control matrix
        self.B = np.zeros((6, 6))
        
        # (x) Initial state estimate vector
        self.x = None  # Must be initialized on first measurement
        
        # (Q) Process noise matrix
        self.Q = np.zeros((6, 6))
        
        # (P0) Initial error covariance matrix
        self.P = self.pos * np.eye(6)
        
        # (H) Measurement matrix
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ])
        
        # (R) Measurement noise matrix
        self.R = self.pos * np.eye(6)
        
        self.initialized = False
    
    def get_location(self):
        """Return state estimation in geodetic coordinates"""
        if not self.initialized:
            return None
        return Point.geocentric_to_geodetic(self.x[0], self.x[2], self.x[4])
    
    def get_point(self):
        """Return state estimation in Cartesian coordinates"""
        if not self.initialized:
            return None
        return Point(self.x[0], self.x[2], self.x[4])
    
    def init_state(self, vector):
        """Initialize the state with a vector"""
        self.x = np.array([
            vector[0], 0.0, vector[2], 0.0, vector[4], 0.0
        ])
        self.initialized = True
    
    def predict(self):
        """Predict step of Kalman filter"""
        # Predict state
        self.x = self.A @ self.x
        
        # Predict error covariance
        self.P = self.A @ self.P @ self.A.T + self.Q
    
    def correct(self, z):
        """Correction step of Kalman filter"""
        # Compute Kalman gain
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        y = z - self.H @ self.x
        self.x = self.x + K @ y
        
        # Update error covariance
        self.P = (np.eye(6) - K @ self.H) @ self.P
    
    def update(self, data, measurement_error=0):
        """Update the filter with new measurement"""
        if isinstance(data, Location):
            point = Point(data.latitude, data.longitude, data.altitude)
            error = data.accuracy
        else:
            point = data
            error = measurement_error
            
        error_scaled = (3 ** -0.5) * error
        z = np.array([
            point.x(), error_scaled,
            point.y(), error_scaled,
            point.z(), error_scaled
        ])
        
        if not self.initialized:
            self.init_state(z)
        else:
            self.predict()
            self.correct(z)