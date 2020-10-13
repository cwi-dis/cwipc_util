from collections import OrderedDict

# Rubiks cube
RH = 0.028 # Half the width/height/depth of the cube

targets=OrderedDict(
    target_v2 = dict(
        description="Cross with forward-point strut and balls",
        points=[
            (0.25, 1, 0, 255, 0, 0),        # Red ball at Right of the cross
            (0, 1.25, 0, 0, 0, 255),        # Blue ball at top of the cross (because the sky is blue)
            (0, 1.1, -0.25, 255, 127, 0),   # Orange ball pointing towards the viewer (-Z) because everyone likes orange
            (-0.25, 1, 0, 255, 255, 0),     # Yellow ball at left of cross because it had to go somewhere
            (0, 0, 0, 0, 0, 0),             # Black point at 0,0,0
            (-1, 0, -1, 0, 0, 0),             # Black point at -1,0,-1
            (-1, 0, 1, 0, 0, 0),             # Black point at -1,0,1
            (1, 0, -1, 0, 0, 0),             # Black point at 1,0,-1
            (1, 0, 1, 0, 0, 0),             # Black point at 1,0,1
            (0, 1, 0, 0, 0, 0),             # Black point at cross
            (0, 1.1, 0, 0, 0, 0),           # Black point at forward spar
        ]
    ),
    target_v3 = dict(
        description="Cross with uneven arms and LEDs",
        points=[
            (-0.26, 1.04, 0, 255, 0, 0),        # Red LED at left of the cross
            (0, 1.44, 0, 0, 0, 255),        # Blue LED at top of the cross (because the sky is blue)
            (0, 1.31, -0.26, 0, 255, 0),   # Green LED pointing towards the viewer (-Z)
            (0.26, 1.17, 0, 255, 255, 0),     # Yellow LED at right of cross because it had to go somewhere
            (0, 0, 0, 0, 0, 0),             # Black point at 0,0,0
            (-1, 0, -1, 0, 0, 0),             # Black point at -1,0,-1
            (-1, 0, 1, 0, 0, 0),             # Black point at -1,0,1
            (1, 0, -1, 0, 0, 0),             # Black point at 1,0,-1
            (1, 0, 1, 0, 0, 0),             # Black point at 1,0,1
        ]
    ),
    a4 = dict(
        description="A4 paper target held at 1.7m high (bottom of paper)",
        points=[
            (-0.105, 1.7+0.297, 0, 0, 0, 255),  # topleft, blue
            (+0.105, 1.7+0.297, 0, 255, 0, 0),  # topright, red
            (-0.105, 1.7, 0, 255, 0, 255),  # botleft, pink
            (+0.105, 1.7, 0, 255, 255, 0),  # botright, yellow
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
    rubik = dict(
        description="Rubiks cube held at 1m with one face looking forward",
        points=[
            (-RH, 1+RH, 0, 127, 127, 127),  # topleft
            (+RH, 1+RH, 0, 127, 127, 127),  # topright
            (-RH, 1-RH, 0, 127, 127, 127),  # botleft
            (+RH, 1-RH, 0, 127, 127, 127),  # botright
        ]
    ),
    videolat = dict(
        description="Videolat calibration target held at 1.25m high",
        points=[
            (-0.040, 1.275,  0.000, 127, 127, 127),    # topleft back row
            ( 0.040, 1.275,  0.000, 127, 127, 127),    # topright back row
            (-0.015, 1.250, -0.025, 127, 127, 127),    # topleft front row
            ( 0.015, 1.250, -0.025, 127, 127, 127),    # topright front row
            (-0.015, 1.225, -0.025, 127, 127, 127),    # bottomleft front row
            ( 0.015, 1.225, -0.025, 127, 127, 127),    # bottomright front row
            (-0.040, 1.200,  0.000, 127, 127, 127),    # bottomleft back row
            ( 0.040, 1.200,  0.000, 127, 127, 127),    # bottomright back row
        ]
    ),
)
