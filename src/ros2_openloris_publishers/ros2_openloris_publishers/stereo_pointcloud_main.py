#!/usr/bin/env python3
"""Start stereo_pointcloud with the distro OpenCV/NumPy runtime pair."""

import sys


def main():
    # The workspace virtual environment currently carries NumPy 2, while
    # Ubuntu's cv2 extension was built against distro NumPy 1. ROS console
    # scripts use /usr/bin/python3, so prefer that package-declared pair.
    dist_packages = '/usr/lib/python3/dist-packages'
    if dist_packages in sys.path:
        sys.path.remove(dist_packages)
    sys.path.insert(0, dist_packages)

    from ros2_openloris_publishers.stereo_pointcloud_node import main as run
    run()


if __name__ == '__main__':
    main()
