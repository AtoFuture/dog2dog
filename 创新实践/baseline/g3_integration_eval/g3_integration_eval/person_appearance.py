"""Small clothing-colour descriptor from a decoded evidence ROI (not face ID)."""
import math

SCHEMA = 'hsv_halves_v1'
DIMENSIONS = 112


def valid_descriptor(value):
    try:
        vector = [float(x) for x in value['vector']]
        return (value['schema'] == SCHEMA and len(vector) == DIMENSIONS
                and all(math.isfinite(x) and x >= 0 for x in vector)
                and abs(sum(vector)-1.) < .001)
    except (KeyError, TypeError, ValueError):
        return False


def describe(bgr):
    import cv2
    import numpy as np
    if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3 or min(bgr.shape[:2]) < 8:
        return None
    # Centre crop suppresses the detector ROI's background margin.
    h, w = bgr.shape[:2]
    image = cv2.resize(bgr[:, w//8:w-w//8], (32, 64))
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    parts = []
    for half in (hsv[:32], hsv[32:]):
        hs = cv2.calcHist([half], [0, 1], None, [12, 4], [0, 180, 0, 256]).ravel()
        brightness = cv2.calcHist([half], [2], None, [8], [0, 256]).ravel()
        # Equal contributions from hue/saturation and brightness, in both halves.
        parts.extend((hs / hs.sum() / 4).tolist())
        parts.extend((brightness / brightness.sum() / 4).tolist())
    return dict(schema=SCHEMA, vector=parts)


def distance(a, b):
    if not valid_descriptor(a) or not valid_descriptor(b):
        return None
    return math.sqrt(max(0., 1.-sum(math.sqrt(float(x)*float(y)) for x, y in zip(a['vector'], b['vector']))))
