from collections import OrderedDict

targets=OrderedDict(

    a4floor = dict(
        description="A4 paper target on the floor, forward-facing side up",
        points=[
            (+0.105, -0.01, -0.148, 0, 0, 255),  # topleft, blue
            (-0.105, -0.01, -0.148, 255, 0, 0),  # topright, red
            (+0.105, -0.01, +0.148, 255, 0, 255),  # botleft, pink
            (-0.105, -0.01, +0.148, 255, 255, 0),  # botright, yellow
        ]
    ),
    a41m = dict(
        description="A4 paper target held at 1m high (bottom of paper)",
        points=[
            (-0.105, 1+0.297, 0, 0, 0, 255),  # topleft, blue
            (+0.105, 1+0.297, 0, 255, 0, 0),  # topright, red
            (-0.105, 1, 0, 255, 0, 255),  # botleft, pink
            (+0.105, 1, 0, 255, 255, 0),  # botright, yellow
        ]
    ),
    a41m0 = dict(
        description="A4 paper target held at 1m high (bottom of paper) and origin",
        points=[
            (-0.105, 1+0.297, 0, 0, 0, 255),  # topleft, blue
            (+0.105, 1+0.297, 0, 255, 0, 0),  # topright, red
            (-0.105, 1, 0, 255, 0, 255),  # botleft, pink
            (+0.105, 1, 0, 255, 255, 0),  # botright, yellow
            (0, 0, 0, 0, 0, 0), # origin, black
        ]
    ),
    a4head = dict(
        description="A4 paper target held at 1.7m high (bottom of paper)",
        points=[
            (-0.105, 1.7+0.297, 0, 0, 0, 255),  # topleft, blue
            (+0.105, 1.7+0.297, 0, 255, 0, 0),  # topright, red
            (-0.105, 1.7, 0, 255, 0, 255),  # botleft, pink
            (+0.105, 1.7, 0, 255, 255, 0),  # botright, yellow
        ]
    ),    
)
