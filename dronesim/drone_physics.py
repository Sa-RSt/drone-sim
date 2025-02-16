import numpy as np
from scipy.spatial.transform import Rotation
import opensimplex
from time import perf_counter


GRAVITY = 9.80665


class DronePhysics:
    def __init__(self):
        self.rot = Rotation.from_euler('xyz', (0, 0, 0))
        self.position = np.zeros(3, dtype=np.float64)
        self.velocity = np.zeros(3, dtype=np.float64)
        self.mass = .4
        self.angular_velocity = Rotation.from_euler('xyz', (0, 0, 0))
        self.moment_of_inertia = 7e-5
        self.motors = np.ones(4, dtype=np.float64)
        self.resistive_drag = 0.015
        self.radius = .15
        self.air_density_randomness = 0.
        self.motor_imperfections = np.random.random(4)
        self.motor_imperfections_scale = 0.
        self._start = perf_counter()

    def total_force(self, dt, densities) -> np.ndarray:
        grav = np.array([0., 0., -self.mass * GRAVITY])
        modulus = np.dot(self.motors, densities) + (self.motor_imperfections_scale * self.motor_imperfections).sum()
        normal = self.rot.apply(np.array([0., 0., modulus]))
        vsquared = np.dot(self.velocity, self.velocity)
        if abs(vsquared) > 1e-9:
            air_res = self.resistive_drag * vsquared
            norm = np.sqrt(vsquared)
            if air_res*dt >= norm:
                air_res = 0.5 * norm / dt
            air_res_dir = -self.velocity / norm
        else:
            air_res = 0
            air_res_dir = np.zeros(3)
        return grav + normal + air_res*air_res_dir

    def total_torque(self, dt, densities) -> np.ndarray:
        base = self.radius * np.array([-np.sqrt(2)/2, np.sqrt(2)/2, 0.])
        forces = self.motors * densities + self.motor_imperfections * self.motor_imperfections_scale
        r0 = self.rot.apply(Rotation.from_euler('xyz', (0, 0, 0)).apply(base))
        r1 = self.rot.apply(Rotation.from_euler('xyz', (0, 0, -np.pi/2)).apply(base))
        r2 = self.rot.apply(Rotation.from_euler('xyz', (0, 0, -np.pi)).apply(base))
        r3 = self.rot.apply(Rotation.from_euler('xyz', (0, 0, -3*np.pi/2)).apply(base))
        f0 = self.rot.apply(Rotation.from_euler('xyz', (0, 0, 0)).apply(np.array([0., 0., forces[0]])))
        f1 = self.rot.apply(Rotation.from_euler('xyz', (0, 0, -np.pi/2)).apply(np.array([0., 0., forces[1]])))
        f2 = self.rot.apply(Rotation.from_euler('xyz', (0, 0, -np.pi)).apply(np.array([0., 0., forces[2]])))
        f3 = self.rot.apply(Rotation.from_euler('xyz', (0, 0, -3*np.pi/2)).apply(np.array([0., 0., forces[3]])))
        motor_torque = np.cross(r0.ravel(), f0.ravel()) + np.cross(r1.ravel(), f1.ravel()) + np.cross(r2.ravel(), f2.ravel()) + np.cross(r3.ravel(), f3.ravel())
        rv = self.angular_velocity.as_rotvec()
        omega2 = np.dot(rv, rv)
        if abs(omega2) > 1e-9:
            air_res = 0.5 * self.resistive_drag * omega2 * self.radius * self.radius
            omega = np.sqrt(omega2)
            if air_res * dt >= omega:
                air_res = 0.5 * omega / dt
            air_res_dir = -rv / omega
        else:
            air_res = 0.
            air_res_dir = np.zeros(3)
        return motor_torque + air_res * air_res_dir

    def tick(self, dt):
        densities = 1. + self.air_density_randomness * opensimplex.noise2array(np.linspace(0, 4*self.radius, 4), np.array([perf_counter()*4] * 4))[0]

        self.tick_linear(dt, densities)
        self.tick_angular(dt, densities)

    def tick_linear(self, dt, densities):
        F = self.total_force(dt, densities)
        self.position += dt * self.velocity / 2
        self.velocity += dt * F / self.mass
        self.position += dt * self.velocity / 2

    def tick_angular(self, dt, densities):
        #breakpoint()
        av = self.angular_velocity.as_rotvec() * dt
        self.rot = Rotation.from_matrix(Rotation.from_rotvec(av).as_matrix() @ self.rot.as_matrix())
        tau = dt * self.total_torque(dt, densities) / self.moment_of_inertia
        self.angular_velocity = Rotation.from_matrix(Rotation.from_rotvec(tau).as_matrix() @ self.angular_velocity.as_matrix())
        av = self.angular_velocity.as_rotvec() * dt
        self.rot = Rotation.from_matrix(Rotation.from_rotvec(av).as_matrix() @ self.rot.as_matrix())
    
    def __repr__(self):
        z, y, x = self.rot.as_euler('zyx')
        return '\n'.join((
            f'X: {self.position[0]:.4f}m  Y: {self.position[1]:.4f}m  Z: {self.position[2]:.4f}m',
            f'dX/dt: {self.velocity[0]:.4f}m/s  dY/dt: {self.velocity[1]:.4f}m/s  dZ/dt: {self.velocity[2]:.4f}m/s',
            f'Z: {z * 180 / np.pi:.4f}º  Y: {y * 180 / np.pi:.4f}º  X: {x * 180 / np.pi:.2f}º'
        ))
