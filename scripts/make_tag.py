import apriltag
import numpy as np
from PIL import Image

# You need a generator function; simplest is to use apriltag library's reference pattern
# If your apriltag package can't generate, use the pre-made images from apriltag GitHub

# Alternative: manually create a tag using pre-made pattern
# For tag36h11 ID 0, you can download the PNG from:
# https://april.eecs.umich.edu/media/apriltags/tag36_11_00000.png

# Then resize to high resolution
img = Image.open("models/ar_marker_0/materials/textures/ar_marker_0.png")
img = img.resize((512, 512), Image.NEAREST)  # keep edges crisp
img.save("ar_marker_0_highres.png")
