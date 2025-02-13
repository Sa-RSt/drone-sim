from dronesim.asset_manager import AssetManager
import pygame
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation
import cv2

def render(asset: AssetManager, yaw: float, pitch: float, roll: float, altitude: float) -> pygame.Surface:
    original = asset.load_img('drone-body.png')
    L = original.size[0]
    yawed = original.rotate(yaw * 180 / np.pi)
    scale_factor = altitude/20
    w = int(L * scale_factor)
    w = max(w, 1.)
    altituded = yawed.resize(tuple(map(int, (w, w))))
    rot = Rotation.from_euler('zyx', (0, pitch, roll))
    edgepoints = [np.array(pt, dtype=np.float64) for pt in [
        (0, 0, 0),
        (altituded.size[0], 0, 0),
        (0, altituded.size[1], 0),
        (altituded.size[0], altituded.size[1], 0),
    ]]
    edgepoints_trans = np.array([rot.apply(pt)[:2] for pt in edgepoints], dtype=np.float32)
    edgepoints_source = np.array([pt[:2] for pt in edgepoints], dtype=np.float32)
    mins = edgepoints_trans.min(axis=0)
    edgepoints_trans -= np.array([[mins[0], mins[1]]] * 4, dtype=np.float32)
    bbox = edgepoints_trans.max(axis=0)
    #breakpoint()
    trans = cv2.getPerspectiveTransform(edgepoints_source, edgepoints_trans)
    perspectived_cv = cv2.warpPerspective(np.array(altituded), trans, tuple(map(round, bbox)), borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
    aliased = Image.fromarray(perspectived_cv)
    r, g, b, a = aliased.split()
    def pointf(v):
        if v < 100:
            v = 0
        return v
    r = r.point(pointf)
    g = g.point(pointf)
    b = b.point(pointf)
    a = a.point(pointf)
    antialiased = Image.merge('RGBA', (r, g, b, a))
    done = pygame.image.frombuffer(antialiased.tobytes(), antialiased.size, 'RGBA')
    done.set_colorkey((0, 0, 0))
    return done.convert()

