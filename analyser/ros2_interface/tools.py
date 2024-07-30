"""
there are some necessary rosbag2 functions in this module
"""

import os
from rosbags.typesys import get_types_from_msg, register_types
from pathlib import Path


def register_new_types(directory_path: str) -> None:
    """
        this function registers new ros2 types
        ! Note:
        # Type import works only after the register_types call,
        # the classname is derived from the msgtype names above.
        # pylint: disable=no-name-in-module,wrong-import-position
        :param directory_path:
        :return:
        """

    path_list = os.listdir(directory_path)
    add_types = {}

    for pathstr in path_list:
        msgpath = Path(directory_path + "/" + pathstr)
        msgdef = msgpath.read_text(encoding='utf-8')
        msgtype = msgpath.relative_to(msgpath.parents[2]).with_suffix('')
        add_types.update(get_types_from_msg(msgdef, msgtype))

    register_types(add_types)
