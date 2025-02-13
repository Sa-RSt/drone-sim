import pygame
from dronesim.asset_manager import AssetManager
from dronesim.drone_physics import DronePhysics, GRAVITY
import numpy as np
from time import perf_counter, sleep
import subprocess
from dronesim import drone_display
from dronesim.slider import Slider
from dronesim.pid import Controller
from scipy.spatial.transform import Rotation
from enum import Enum


class Mode(Enum):
    PID = 1
    MANUAL = 0


pygame.display.init()
pygame.font.init()

MIN_THRUST = 0.
MAX_THRUST = 1.5
DEFAULT_POSITION = np.array([0., 0., 20.])

dp = DronePhysics()
dp.motors = np.array([0.] * 4)
dp.position = DEFAULT_POSITION.copy()
dp.angular_velocity = Rotation.from_rotvec(np.array([0., 0., 0.]))
motor_color = (250, 100, 90)
motor_sliders = [
    Slider(
        color=motor_color,
        deactivated_color=(100, 100, 100),
        minimum=MIN_THRUST,
        maximum=MAX_THRUST,
        value=GRAVITY * dp.mass / 4,
        width=550,
        height=50,
        decimal_places=2,
        rendered_text=pygame.font.SysFont('sans-serif', 25).render(f'Motor {i}', True, motor_color, (255, 255, 255)),
        value_renderer=lambda x: pygame.font.SysFont('sans-serif', 16).render(f'{1000 * x:.3f} (Δ={abs(1000 * (x - dp.motors[0])):.3f})', True, motor_color, (255, 255, 255))
    ) for i in range(4)
]

adr_color = (90, 100, 250)
adr_slider = Slider(
    color=adr_color,
    deactivated_color=(100, 100, 100),
    minimum=0.,
    maximum=.001,
    value=0.,
    width=550,
    height=50,
    decimal_places=2,
    rendered_text=pygame.font.SysFont('sans-serif', 25).render(f'ADA', True, adr_color, (255, 255, 255)),
    value_renderer=lambda x: pygame.font.SysFont('sans-serif', 20).render(f'{100*x:.3f} %', True, adr_color, (255, 255, 255))
)
print(dp)

screen = pygame.display.set_mode((1137, 840))
screen.set_colorkey((0, 255, 0))

am = AssetManager()

pos_control = Controller(1.3, 1.8, .2, 3)
vel_control = Controller(2., .1, 1.2, 3)
ang_control = Controller(4., 3, .1, 3)
speed = np.zeros(3, dtype=np.float64)

mode = Mode.MANUAL

#exit()
old_time = perf_counter()
while 1:
    new_time = perf_counter()
    dt = new_time - old_time
    dp.tick(dt)
    old_time = new_time
    print(dp)
    print(f'{1/dt:.2f} Hz')
    sliderx = screen.get_width() - 550
    
    
    for evt in pygame.event.get():
        slidery = 50
        for slider in motor_sliders:
            slider.tick(sliderx, slidery, evt)
            slidery += 50
        adr_slider.tick(sliderx, slidery, evt)
        if evt.type == pygame.QUIT:
            exit()
        elif evt.type == pygame.KEYDOWN:
            effect = .1
            if evt.key == pygame.K_ESCAPE:
                exit()
            elif evt.key == pygame.K_m:
                mode = Mode.MANUAL
            elif evt.key == pygame.K_p:
                mode = Mode.PID
            elif evt.key == pygame.K_UP:
                speed[1] += +1. * effect
            elif evt.key == pygame.K_DOWN:
                speed[1] += -1. * effect
            elif evt.key == pygame.K_LEFT:
                speed[0] += -1. * effect
            elif evt.key == pygame.K_RIGHT:
                speed[0] += +1. * effect
            elif evt.key == pygame.K_q:
                speed[2] += +3. * effect
            elif evt.key == pygame.K_e:
                speed[2] += -3. * effect
            elif evt.key == pygame.K_r:
                dp.position = DEFAULT_POSITION.copy()
        """ elif evt.type == pygame.KEYUP:
            if evt.key == pygame.K_UP:
                desired_position[1] -= +1.
            elif evt.key == pygame.K_DOWN:
                desired_position[1] -= -1.
            elif evt.key == pygame.K_LEFT:
                desired_position[0] -= +1.
            elif evt.key == pygame.K_RIGHT:
                desired_position[0] -= -1.
            elif evt.key == pygame.K_q:
                desired_position[2] -= +1.
            elif evt.key == pygame.K_e:
                desired_position[2] -= -1. """
    yaw, pitch, roll = dp.rot.as_euler('zyx')
    surf = drone_display.render(am, yaw, pitch, roll, dp.position[2])
    centerx = surf.get_width() / 2
    centery = surf.get_height() / 2
    screen.fill((255, 255, 255))
    screen.blit(surf, (dp.position[0]*100 - centerx + screen.get_width()/2, screen.get_height()/2 - dp.position[1]*100 - centery))
    slidery = 50
    for slider in motor_sliders:
        screen.blit(slider.render(), (sliderx, slidery))
        slidery += 50
    screen.blit(adr_slider.render(), (sliderx, slidery))
    for i, slider in enumerate(motor_sliders):
        dp.motors[i] = slider.value
    dp.air_density_randomness = adr_slider.value

    if mode == Mode.PID:
        pos_error = speed - dp.velocity
        m_pos_error = np.linalg.norm(pos_error)
        if m_pos_error > 3:
            pos_error = 4 * pos_error / m_pos_error
        linear_acc = vel_control.feedback(pos_error, dt) + np.array([0., 0., GRAVITY])
        m_linear_acc = np.linalg.norm(linear_acc)
        if abs(m_linear_acc) > 1e-9:
            yaw, pitch, roll = dp.rot.as_euler('zyx')
            xangle = np.asin(min(max(-linear_acc[1] / m_linear_acc / np.cos(pitch), -1.), 1.))
            yangle = np.asin(min(max(linear_acc[0] / m_linear_acc, -1.), 1.))
        else:
            xangle = 0.
            yangle = 0.
        #input()
        #if xangle > np.pi/12 or yangle > np.pi/12:
        #    vec = np.array([xangle, yangle])
        #    xangle, yangle = (np.pi/12) * vec / np.linalg.norm(vec)
        rv = Rotation.from_euler('xyz', (xangle, yangle, 0)).as_rotvec()
        angular_acc = Rotation.from_rotvec(ang_control.feedback(rv - dp.rot.as_rotvec(), dt))
        xacc, yacc, _ = angular_acc.as_euler('xyz')
        thrust = linear_acc[2] * dp.mass / 4
        I = dp.moment_of_inertia
        dp.motors = np.array([
            max(min(thrust + I*xacc/dp.radius/2 + I*yacc/dp.radius/2, MAX_THRUST), MIN_THRUST),
            max(min(thrust + I*xacc/dp.radius/2 - I*yacc/dp.radius/2, MAX_THRUST), MIN_THRUST),
            max(min(thrust - I*xacc/dp.radius/2 - I*yacc/dp.radius/2, MAX_THRUST), MIN_THRUST),
            max(min(thrust - I*xacc/dp.radius/2 + I*yacc/dp.radius/2, MAX_THRUST), MIN_THRUST)
        ])
        for i, slider in enumerate(motor_sliders):
            slider.value = dp.motors[i]
            slider.enabled = False
        t = f'Velocidade alvo: dx/dt = {speed[0]:3.3f} m/s, dy/dt = {speed[1]:3.3f} m/s, dz/dt = {speed[2]:3.3f} m/s'
        r = pygame.font.SysFont('monospace', 15).render(t, True, (0, 0, 0))
        screen.blit(r, (10, screen.get_height() - 30))
    else:
        speed = np.zeros(3, dtype=np.float64)
        for slider in motor_sliders:
            slider.enabled = True
    pygame.display.flip()

