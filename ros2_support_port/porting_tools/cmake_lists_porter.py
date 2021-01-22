# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License").
# You may not use this file except in compliance with the License.
# A copy of the License is located at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# or in the "license" file accompanying this file. This file is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied. See the License for the specific language governing
# permissions and limitations under the License.

"""
A class for porting CMakeLists from ROS1 to ROS2
"""
import logging
try:
    import parse_cmake.parsing as cmkp
except ImportError:
    import traceback
    traceback.print_exc()
    print("ERROR: You must install the parse_cmake python module!")
    print("")
    print("pip3 install parse_cmake")
    exit(1)
try:
    from ament_index_python import get_packages_with_prefixes
    KNOWN_ROS2_PKGS = get_packages_with_prefixes().keys()
except ImportError:
    KNOWN_ROS2_PKGS = []
    logging.info("Source a ROS2 workspace to get more info about missing \
        packages. Without, all packages will be listed as missing.")

from .constants import (
    RENAMED_ROS_PACKAGES,
    NO_CMAKE_FIND_PACKAGES,
    CMAKE_LISTS_DELETED_COMMANDS,
    CMAKE_LISTS_RENAMED_VARIABLES
)


def _commands_with(name, from_cmake, start=0, end=-1):
    """
    Returns a list of all cmkp._Command objects from a cmakeLists
    with the given name.
    """
    cmd_list = []
    for (index, command) in enumerate(from_cmake[start:end]):
        if isinstance(command, cmkp._Command) and command.name == name:
            yield (index, command)


def _delete_commands_with(name, from_cmake, start=0, end=-1):
    """
    Deletes commands with a given name in a given range.
    """
    def is_removable(pair):
        is_command = isinstance(pair[1], cmkp._Command)
        return is_command and pair[1].name == name
    relevant_pairs = filter(is_removable, enumerate(from_cmake[start:end]))
    remove_indices = [pair[0] for pair in relevant_pairs]
    for index in sorted(remove_indices, reverse=True):
        del from_cmake[index]


def _delete_command_idxs(indexes, from_cmake):
    """
    Delete a command of a specific index.
    """
    for index in sorted(indexes, reverse=True):
        del from_cmake[index]


class CMakeListsPorter:
    def __init__(self, src):
        self.src = src
        with open(src, "r") as fsrc:
            self.contents = fsrc.read()
        self.unknown_packages = []
        self.renamed_packages = []
        self.removed_packages = []
        self.removable_idxs = []
        self.insert_after = []
        self.removed_commands = []
        self.problems = []

    def port(self, dst, extra_rules=[]):
        self.cmake = cmkp.parse(self.contents)
        rules = [
            self.rule_set_cmake_version,
            self.rule_update_dependencies,
            self.rule_remove_unused_commands,
            self.rule_add_ament_lint_auto
        ]
        for rule in rules + extra_rules:
            rule(self.cmake)

        # This command must be at the bottom of the package
        self.cmake.append(cmkp.BlankLine())
        ament_package_cmd = cmkp.Command("ament_package", [])
        self.cmake.append(ament_package_cmd)

        cmake_contents = self.cmake.pretty_print()
        # Replace variable names:
        for var, replacement in CMAKE_LISTS_RENAMED_VARIABLES.items():
            cmake_contents = cmake_contents.replace(var, replacement)

        with open(dst, "w") as fdst:
            fdst.write(cmake_contents)

    #########################
    #        RULES          #
    #########################
    def rule_set_cmake_version(self, cmake):
        """
        Set the cmake version to 3.5
        """
        try:
            idx, cmk_min_req = next(
                _commands_with("cmake_minimum_required", cmake)
            )
            cmake[idx] = cmake[idx]._replace(
                body=[cmkp.Arg("VERSION"), cmkp.Arg("3.5")]
            )
        except StopIteration:
            cmake.insert(
                0,
                cmkp.Command(
                    "cmake_minimum_required",
                    [cmkp.Arg("VERSION"), cmkp.Arg("3.5")]
                )
            )

    def rule_update_dependencies(self, cmake):
        """
        Update package dependencies to what works with ament.
        Assumes the CMakeLists is fairly easy.
        Follow RENAMED_ROS_PACKAGES and NO_CMAKE_FIND_PACKAGES.
        """
        find_pkgs = [p for p in _commands_with("find_package", cmake)]
        removable_idxs = []
        for idx, command in find_pkgs:
            if command.body[0].contents in RENAMED_ROS_PACKAGES.keys():
                new_name = RENAMED_ROS_PACKAGES[command.body[0].contents]
                self.renamed_packages.append(
                    command.body[0].contents + " -> " + new_name
                )
                cmake[idx] = cmake[idx]._replace(
                    body=[cmkp.Arg(new_name)] + cmake[idx].body[1:]
                )
            elif command.body[0].contents in NO_CMAKE_FIND_PACKAGES:
                self.removed_packages.append(command.body[0].contents)
                removable_idxs.append(idx)
            elif command.body[0].contents not in KNOWN_ROS2_PKGS:
                self.unknown_packages.append(command.body[0].contents)
        _delete_command_idxs(removable_idxs, cmake)

    def rule_remove_unused_commands(self, cmake):
        """
        Removes commands that aren't necessary in ament.
        The commands to remove are listed in constants.py
        """
        removable_idxs = []
        for command_name in CMAKE_LISTS_DELETED_COMMANDS:
            for p in _commands_with(command_name, cmake):
                self.removed_commands.append(
                    cmkp.command_to_lines(
                        p[1],
                        cmkp.FormattingOptions()
                    )
                )
                removable_idxs.append(p[0])
        _delete_command_idxs(removable_idxs, cmake)

    def rule_add_ament_lint_auto(self, cmake):
        ament_lint_added = False
        # Identify if_blocks
        self.find_if_blocks()
        removable_idxs = []
        insertion_pairs = []
        for start, stop in self.if_blocks:
            condition = self.cmake[start].body
            if "CATKIN_ENABLE_TESTING" in [arg.contents for arg in condition]:
                # Mark the lines for deletion
                removable_idxs += [a for a in range(start, stop+1)]
        # Delete
        _delete_command_idxs(removable_idxs, cmake)
        cmake.append(
            cmkp.Command(
                "if",
                [cmkp.Arg("BUILD_TESTING")]
            )
        )
        cmake.append(
            cmkp.Command(
                "find_package",
                [cmkp.Arg("ament_lint_auto"), cmkp.Arg("REQUIRED")]
            )
        )
        cmake.append(
            cmkp.Command(
                "ament_lint_auto_find_test_dependencies",
                []
            )
        )
        cmake.append(
            cmkp.Command(
                "endif",
                []
            )
        )

    #########################
    #          HELPERS      #
    #########################
    def find_if_blocks(self):
        """
        Populate self.if_blocks, with index pairs for start and stop of
        the if blocks.
        """
        if_idxs = [p[0] for p in _commands_with("if", self.cmake)]
        endif_idxs = [p[0] for p in _commands_with("endif", self.cmake)]
        self.if_blocks = list(zip(if_idxs, endif_idxs))

    def generate_report(self):
        rep = ""
        rem_pkgs = len(self.removed_packages)
        mis_pkgs = len(self.unknown_packages)
        ren_pkgs = len(self.renamed_packages)
        if rem_pkgs + mis_pkgs + ren_pkgs > 0:
            rep += f"CMakeLists.txt file: {self.src}\n"
            if mis_pkgs > 0:
                rep += "Missing packages:\n"
                rep += "\t" + "\n\t".join(self.unknown_packages)
                rep += "\n"
            if rem_pkgs > 0:
                rep += "Removed packages:\n"
                rep += "\t" + "\n\t".join(self.removed_packages)
                rep += "\n"
            if ren_pkgs > 0:
                rep += "Renamed packages:\n"
                rep += "\t" + "\n\t".join(self.renamed_packages)
                rep += "\n"
            rep += "\n"
        return rep


if __name__ == "__main__":
    src = "/home/mathia/Programming/ros1_ws/src/motoman_gp7_support/CMakeLists.txt"
    conv = CMakeListsPorter(src)
    conv.port("./updated_CMakeLists.txt")
