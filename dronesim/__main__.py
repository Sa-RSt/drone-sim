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
from pathlib import Path
import seaborn as sns
import pandas as pd
import matplotlib.pyplot as plt
import datetime


class Mode(Enum):
    PID = 1
    MANUAL = 0


def make_lists(n):
    for _ in range(n):
        yield []


pygame.display.init()
pygame.font.init()
sns.set_style('whitegrid')

MIN_THRUST = 0.
MAX_THRUST = 1.5
DEFAULT_POSITION = np.array([0., 0., 20.])
OUTPUT_FOLDER = Path(__file__).parent.parent / 'out'

OUTPUT_FOLDER.mkdir(exist_ok=True)

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
        rendered_text=pygame.font.SysFont(
            'sans-serif', 25).render(f'Motor {i+1}', True, motor_color, (255, 255, 255)),
        value_renderer=lambda x: pygame.font.SysFont('sans-serif', 16).render(
            f'{1000 * x:.3f} (Δ={abs(1000 * (x - dp.motors[0])):.3f})', True, motor_color, (255, 255, 255))
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
    rendered_text=pygame.font.SysFont(
        'sans-serif', 25).render(f'ADA', True, adr_color, (255, 255, 255)),
    value_renderer=lambda x: pygame.font.SysFont(
        'sans-serif', 20).render(f'{100*x:.3f} %', True, adr_color, (255, 255, 255))
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
recording_start = None
rec_x, rec_y, rec_z, rec_rx, rec_ry, rec_rz, rec_t, rec_m1, rec_m2, rec_m3, rec_m4, rec_dxdt, rec_dydt, rec_dzdt, rec_drxdt, rec_drydt, rec_drzdt = make_lists(
    17)
parameters = [
    ('Posição', [('x', rec_x),
                 ('y', rec_y),
                 ('z', rec_z)]),

    ('Rotação', [('RX', rec_rx),
                 ('RY', rec_ry),
                 ('RZ', rec_rz)]),

    ('Motor SE', [('Motor 1', rec_m1)]),
    ('Motor SD', [('Motor 2', rec_m2)]),
    ('Motor ID', [('Motor 3', rec_m3)]),
    ('Motor IE', [('Motor 4', rec_m4)]),

    ('Velocidade linear', [('dx/dt', rec_dxdt),
                           ('dy/dt', rec_dydt),
                           ('dz_dt', rec_dzdt)]),
    ('Velocidade angular', [('dRX/dt', rec_drxdt),
                            ('dRY/dt', rec_drydt),
                            ('dRZ/dt', rec_drzdt)])
]

recording_surf = pygame.Surface((300, 30))
recording_surf.fill((255, 255, 255))
pygame.draw.circle(recording_surf, (255, 127, 127), (15, 15), 7)
t = pygame.font.SysFont('sans-serif', 20).render('Gravando',
                                                 True, (255, 127, 127), (255, 255, 255))
recording_surf.blit(t, (30, 15 - t.get_height()/2))

paused_surf = pygame.Surface(screen.get_size()).convert_alpha()
paused_surf.fill((0, 0, 0, 50))

# exit()
old_time = perf_counter()
effect = .1
paused = False
while 1:
    new_time = perf_counter()
    dt = new_time - old_time
    if not paused:
        dp.tick(dt)
    old_time = new_time
    print(dp)
    print(f'{1/dt:.2f} Hz')

    if recording_start is not None:
        rec_t.append(new_time - recording_start)
        rec_x.append(dp.position[0])
        rec_y.append(dp.position[1])
        rec_z.append(dp.position[2])
        rx, ry, rz = dp.rot.as_euler('xyz')
        rec_rx.append(rx)
        rec_ry.append(ry)
        rec_rz.append(rz)
        rec_m1.append(dp.motors[0])
        rec_m2.append(dp.motors[1])
        rec_m3.append(dp.motors[2])
        rec_m4.append(dp.motors[3])
        rec_dxdt.append(dp.velocity[0])
        rec_dydt.append(dp.velocity[1])
        rec_dzdt.append(dp.velocity[2])
        rx, ry, rz = dp.angular_velocity.as_euler('xyz')
        rec_drxdt.append(rx)
        rec_drydt.append(ry)
        rec_drzdt.append(rz)

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
            if evt.key == pygame.K_ESCAPE:
                exit()
            elif evt.key == pygame.K_LSHIFT:
                effect = 1.
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
            elif evt.key == pygame.K_x:
                paused = not paused
            elif evt.key == pygame.K_SPACE:
                if recording_start is None:
                    recording_start = perf_counter()
                else:
                    recording_start = None
                    now = datetime.datetime.now().isoformat().replace(
                        ':', '_').replace('/', '_').replace('.', '_')
                    tf = pd.Series(rec_t).to_frame('Tempo (s)')
                    for title, charts in parameters:
                        fig, ax = plt.subplots()
                        tf[charts[0][0]] = charts[0][1]
                        ax = sns.lineplot(tf, x='Tempo (s)',
                                          y=charts[0][0], ax=ax)
                        tf.drop(labels=[charts[0][0]], axis=1, inplace=True)
                        for label, vals in charts[1:]:
                            tf[label] = vals
                            ax = sns.lineplot(
                                tf, x='Tempo (s)', y=label, ax=ax)
                            tf.drop(labels=[label], axis=1, inplace=True)
                        ax.set_title(title)
                        fig.savefig(
                            (OUTPUT_FOLDER / f'{title} - {now}').with_suffix('.png'))

        elif evt.type == pygame.KEYUP:
            if evt.key == pygame.K_LSHIFT:
                effect = .1
    yaw, pitch, roll = dp.rot.as_euler('zyx')
    surf = drone_display.render(am, yaw, pitch, roll, dp.position[2])
    centerx = surf.get_width() / 2
    centery = surf.get_height() / 2
    screen.fill((255, 255, 255))
    screen.blit(surf, (dp.position[0]*100 - centerx + screen.get_width() /
                2, screen.get_height()/2 - dp.position[1]*100 - centery))
    slidery = 50
    for slider in motor_sliders:
        screen.blit(slider.render(), (sliderx, slidery))
        slidery += 50
    screen.blit(adr_slider.render(), (sliderx, slidery))
    for i, slider in enumerate(motor_sliders):
        dp.motors[i] = slider.value
    dp.air_density_randomness = adr_slider.value

    if not paused:
        if mode == Mode.PID:
            pos_error = speed - dp.velocity
            m_pos_error = np.linalg.norm(pos_error)
            if m_pos_error > 3:
                pos_error = 4 * pos_error / m_pos_error
            linear_acc = vel_control.feedback(
                pos_error, dt) + np.array([0., 0., GRAVITY])
            m_linear_acc = np.linalg.norm(linear_acc)
            if abs(m_linear_acc) > 1e-9:
                yaw, pitch, roll = dp.rot.as_euler('zyx')
                xangle = np.asin(
                    min(max(-linear_acc[1] / m_linear_acc / np.cos(pitch), -1.), 1.))
                yangle = np.asin(min(max(linear_acc[0] / m_linear_acc, -1.), 1.))
            else:
                xangle = 0.
                yangle = 0.
            # input()
            # if xangle > np.pi/12 or yangle > np.pi/12:
            #    vec = np.array([xangle, yangle])
            #    xangle, yangle = (np.pi/12) * vec / np.linalg.norm(vec)
            rv = Rotation.from_euler('xyz', (xangle, yangle, 0)).as_rotvec()
            angular_acc = Rotation.from_rotvec(
                ang_control.feedback(rv - dp.rot.as_rotvec(), dt))
            xacc, yacc, _ = angular_acc.as_euler('xyz')
            thrust = linear_acc[2] * dp.mass / 4
            I = dp.moment_of_inertia
            dp.motors = np.array([
                max(min(thrust + I*xacc/dp.radius/2 + I *
                    yacc/dp.radius/2, MAX_THRUST), MIN_THRUST),
                max(min(thrust + I*xacc/dp.radius/2 - I *
                    yacc/dp.radius/2, MAX_THRUST), MIN_THRUST),
                max(min(thrust - I*xacc/dp.radius/2 - I *
                    yacc/dp.radius/2, MAX_THRUST), MIN_THRUST),
                max(min(thrust - I*xacc/dp.radius/2 + I *
                    yacc/dp.radius/2, MAX_THRUST), MIN_THRUST)
            ])
            for i, slider in enumerate(motor_sliders):
                slider.value = dp.motors[i]
                slider.enabled = False
        else:
            speed = np.zeros(3, dtype=np.float64)
            for slider in motor_sliders:
                slider.enabled = True
    if mode == Mode.PID:
        t = f'Velocidade alvo: dx/dt = {speed[0]:3.3f} m/s, dy/dt = {speed[1]:3.3f} m/s, dz/dt = {speed[2]:3.3f} m/s'
        r = pygame.font.SysFont('monospace', 15).render(t, True, (0, 0, 0))
        screen.blit(r, (10, screen.get_height() - 30))
    if recording_start is not None:
        screen.blit(recording_surf, (10, 10))
    if paused:
        screen.blit(paused_surf, (0, 0))
    pygame.display.flip()
