"""ROS 2 adapter: odometry in, velocity commands and diagnostics out."""

from .ros_runtime import run


def main(args=None) -> None:
    run(args)


if __name__ == "__main__":
    main()

