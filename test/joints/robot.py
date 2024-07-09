import numpy as np
from roboticstoolbox.robot.ET import ET
from roboticstoolbox.robot.Robot import Robot
from roboticstoolbox.robot.Link import Link


class Parol6(Robot):

    def __init__(self):

        deg = np.pi / 180

        l1 = Link(
            ET.Rz(),
            qlim=(-180*deg, 180*deg),
            name="L1",
            parent=None
        )
        l2 = Link(
            ET.tx(0.02342) * ET.tz(0.1105) * ET.Rx(-90 * deg) * ET.Rz(),
            qlim=(-90*deg, 90*deg),
            name="L2",
            parent=l1
        )
        l3 = Link(
            ET.ty(-0.18) * ET.Rz(-90 * deg) * ET.Rx(-180 * deg) * ET.Rz(),
            qlim=(-90*deg, 90*deg),
            name="L3",
            parent=l2
        )
        l4 = Link(
            ET.tx(0.0435) * ET.Rz(180 * deg) * ET.Rx(-90 * deg) * ET.Rz(),
            qlim=(-15*deg, 104*deg),
            name="L4",
            parent=l3
        )
        l5 = Link(
            ET.tz(0.1764) * ET.Rx(90 * deg) * ET.Rz(),
            qlim=(-90*deg, 90*deg),
            name="L5",
            parent=l4,
        )
        l6 = Link(
            ET.ty(0.039) * ET.Rx(-90 * deg) * ET.Rz(),
            qlim=(-180*deg, 180*deg),
            name="L6",
            parent=l5,
        )

        elinks = [l1, l2, l3, l4, l5, l6]

        super().__init__(elinks, name="Parol6")

        self.qr = np.array([0, 0, 0, 0, 0, 90 * deg])
        self.addconfiguration("qr", self.qr)


def make_robot():
    return Parol6()
