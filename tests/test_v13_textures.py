"""v13 #2: robot meshes reach the browser with textures of at most TEXTURE_PX (PNG encoding of the
UR 2048 px atlases was ~10 s of every app start); meshes and texture coordinates are unchanged."""

from functools import partial

import yourdfpy
from PIL import Image

from core.robot import Robot
from ui.robot_view import TEXTURE_PX, light_urdf


def test_textures_are_downscaled_and_meshes_kept():
    spec = Robot.from_registry("ur20").spec
    path = spec.folder / spec.urdf
    handler = partial(yourdfpy.filename_handler_magic, dir=path.parent)
    full = yourdfpy.URDF.load(path, build_scene_graph=True, load_meshes=True, filename_handler=handler)
    light = light_urdf(path)
    assert set(light.scene.geometry) == set(full.scene.geometry)
    big = 0
    for name, g in light.scene.geometry.items():
        im = g.visual.material.baseColorTexture
        assert isinstance(im, Image.Image) and max(im.size) <= TEXTURE_PX
        assert g.visual.uv.shape == full.scene.geometry[name].visual.uv.shape
        big += max(full.scene.geometry[name].visual.material.baseColorTexture.size) > TEXTURE_PX
    assert big > 0  # the UR20 atlas is 2048 px: the test covers a real downscale
    again = light_urdf(path)  # the small copies are reused
    k = next(iter(again.scene.geometry))
    assert (
        again.scene.geometry[k].visual.material.baseColorTexture
        is light.scene.geometry[k].visual.material.baseColorTexture
    )
