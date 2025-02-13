import PIL.Image
import pygame
from pathlib import Path
import PIL

ASSET_ROOT = Path(__file__).parent.parent / 'assets'

class AssetManager:
    def __init__(self):
        self._assets = {}
    
    def load_img(self, filename) -> PIL.Image.Image:
        if filename in self._assets:
            return self._assets[filename]
        path = (ASSET_ROOT / 'a').with_name(filename)
        img = PIL.Image.open(path)
        self._assets[filename] = img
        return img
