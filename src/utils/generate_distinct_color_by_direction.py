import numpy as np


def generate_directional_color(direction):
    if direction == "x":
        return [1, 0.4, 0.4]
    elif direction == "y":
        return [0.4, 1, 0.4]
    elif direction == "z":
        return [0.4, 0.4, 1]
    else:
        return [0.5, 0.5, 0.5]


def generate_distinct_random_color(direction, seed=None):
    if (direction == "unclassified"):
        return [0.5, 0.5, 0.5]

    if seed is not None:
        np.random.seed(seed)

    # 色相環からランダムに色相を選択
    hue = np.random.random()

    # 彩度と明度を調整
    saturation = 0.7 + np.random.random() * 0.3  # 0.7-1.0
    value = 0.7 + np.random.random() * 0.3       # 0.7-1.0

    # HSVからRGBに変換
    h_i = int(hue * 6)
    f = hue * 6 - h_i
    p = value * (1 - saturation)
    q = value * (1 - f * saturation)
    t = value * (1 - (1 - f) * saturation)

    if h_i == 0:
        return [value, t, p]
    elif h_i == 1:
        return [q, value, p]
    elif h_i == 2:
        return [p, value, t]
    elif h_i == 3:
        return [p, q, value]
    elif h_i == 4:
        return [t, p, value]
    else:
        return [value, p, q]


def generate_unique_color(count):
    hue = (count * 0.618033988749895) % 1
    saturation = 0.7
    value = 0.9

    h_i = int(hue * 6)
    f = hue * 6 - h_i
    p = value * (1 - saturation)
    q = value * (1 - f * saturation)
    t = value * (1 - (1 - f) * saturation)

    if h_i == 0:
        return [value, t, p]
    elif h_i == 1:
        return [q, value, p]
    elif h_i == 2:
        return [p, value, t]
    elif h_i == 3:
        return [p, q, value]
    elif h_i == 4:
        return [t, p, value]
    else:
        return [value, p, q]
