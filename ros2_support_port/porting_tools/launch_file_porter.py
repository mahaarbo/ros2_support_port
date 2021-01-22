import logging
from pathlib import Path
import re
import autopep8
import jinja2
import xml.etree.ElementTree as etree

from .constants import RENAMED_ROS_PACKAGES, RENAMED_ROS_EXECUTABLES
from .launch_actions.launch_action import LaunchAction, LaunchCondition
from .launch_actions.include_launch_description import IncludeLaunchDescription

try:
    from ament_index_python import get_packages_with_prefixes
    KNOWN_ROS2_PKGS = sorted(list(get_packages_with_prefixes().keys()))
except ImportError:
    KNOWN_ROS2_PKGS = []
    logging.info("Source a ROS2 workspace to get more info about missing"
                 " packages. Without it, all packages will be listed as "
                 "missing.")


class LaunchFilePorter:
    """
    A class for porting a launch file from ROS1 to ROS2.
    """
    def __init__(self, src: Path):
        self.src = src
        with open(src, "r") as fsrc:
            self.contents = fsrc.read()
        self.imports = set()
        self.preparatory = []
        self.launch_actions = []
        # Launch Description entity strings, for when things are defined
        # in the preparatory stage, and later referenced f.ex.
        self.ld_entity_strings = []
        self.global_params = []
        self.problems = []

    def port(self, dst: Path, extra_rules=[]):
        """
        Port the launch file.
        """
        # Parse
        self.launch_root = etree.fromstring(self.contents)
        self.launch_tree = etree.ElementTree(self.launch_root)

        # Rules to apply
        rules = [
            self.rule_convert_args,
            self.rule_convert_params,
            self.rule_report_rosparam,
            self.rule_convert_groups,
            self.rule_convert_includes,
            self.rule_convert_nodes,
            self.rule_robot_state_publisher
        ]

        # Perform the rules on the contents
        for rule in rules + extra_rules:
            rule(self.launch_root)

        # Prepare strings for template
        imports_str = "\n".join(self.imports)
        preperatory_str = "\n    ".join(self.preparatory)
        ld_str = self._generate_ld_strings(self.launch_actions)
        # Use jinja template
        template_dir = Path(__file__).parent.joinpath("templates")
        env = jinja2.Environment(
            loader=jinja2.FileSystemLoader(
                searchpath=template_dir
            )
        )
        template = env.get_template("launch_file.j2")
        rendered_template = template.render(
            imports=imports_str,
            preparatory=preperatory_str,
            actions=ld_str
        )
        # pep8ify
        launch_contents = autopep8.fix_code(rendered_template)

        # Write
        if "py" not in dst.suffix:
            dst = dst.with_suffix(dst.suffix + ".py")
        with open(dst, "w") as fdst:
            fdst.write(launch_contents)

    #########################
    #        RULES          #
    #########################
    def rule_convert_args(self, root_elem):
        """
        Convert all <arg> elements to ROS2 launch actions.
        """
        for elem in root_elem.findall("arg"):
            default_value = self._convert_attribute(
                elem.attrib.get("default", None)
            )
            if default_value is None:
                default_value = self._convert_attribute(
                    elem.attrib.get("value", None)
                )
            name = self._convert_attribute(elem.attrib.get("name", None))
            description = self._convert_attribute(elem.attrib.get("doc", None))
            launch_action = LaunchAction(
                "DeclareLaunchArgument",
                {
                    "name": name,
                    "default_value": default_value,
                    "description": description
                }
            )
            self.launch_actions.append(launch_action)

    def rule_convert_params(self, root_elem):
        """
        Convert all <param> tags, attach to all nodes.

        Horrible way of doing it, but works for now.
        """
        param_elems = root_elem.findall("param")
        for elem in param_elems:
            param_name = elem.attrib.get("name")
            # Robot descriptions are special
            if param_name == "robot_description":
                self._convert_robot_description(elem)
                continue
            # General parameters are given to everyone
            elem_value = elem.attrib.get("value", None)
            param_value = self._convert_attribute(elem_value)
            parameter = {}
            parameter[param_name] = param_value
            self.global_params.append(parameter)

    def rule_report_rosparam(self, root_elem):
        """
        Report <rosparam> tags. No handling at the moment.
        """
        first_warn = True
        for elem in root_elem.findall("rosparam"):
            if first_warn:
                warn = "Since loading of parameters has changed. These must " \
                       " be ported by hand."
                self.problems.append(warn)
                first_warn = False
            self.problems.append(
                etree.tostring(elem, encoding="unicode").strip()
            )

    def rule_convert_groups(self, root_elem):
        """
        Convert all <group> elements and their subelements.
        """
        for elem in root_elem.findall("group"):
            group_if_attrib = elem.attrib.get("if", None)
            if group_if_attrib is None:
                warn = "Found group tag without 'if' attribute"
                logging.warning(warn)
                self.problems.append(warn)
                self.problems.append(
                    etree.tostring(elem, encoding="unicode").strip()
                )
                continue
            converted_attrib = self.convert_attribute(group_if_attrib)
            if_condition = LaunchCondition("IfCondition", converted_attrib)
            for node_elem in elem.findall("node"):
                launch_action = self._convert_node_elem(node_elem)
                launch_action.add_condition(if_condition)
            self.launch_actions.append(launch_action)

    def rule_convert_nodes(self, root_elem):
        """
        Convert all <node> elements.
        """
        for elem in root_elem.findall("node"):
            launch_action = self._convert_node_elem(elem)
            self.launch_actions.append(launch_action)

    def rule_convert_includes(self, root_elem):
        """
        Convert all <include> elements.
        """
        for elem in root_elem.findall("include"):
            launch_action = self._convert_include_elem(elem)
            if launch_action is not None:
                self.imports.update(["from ament_index_python.packages import"
                                     " get_package_share_directory"])
                self.launch_actions.append(launch_action)

    def rule_robot_state_publisher(self, root_elem):
        """
        Example of one of the funky rules that can be made.

        Also useful since robot_state_publisher needs robot_description.
        """
        for action in self.launch_actions:
            if action.action_type == "Node":
                if "robot_state_publisher" in action.parameters["executable"]:
                    if "parameters" not in action.parameters.keys():
                        action.parameters["parameters"] = []
                    action.parameters["parameters"].append(
                        "{'robot_description': launch.substitutions."
                        "LaunchConfiguration('robot_description')}"
                    )

    #########################
    #          HELPERS      #
    #########################
    def _convert_attribute(self, value):
        """
        Convert the value of a single attribute
        """
        # Check for empty
        if value is None:
            return None
        # Check if it is an arg
        arg_search = re.search(r"\$\(arg ([a-zA-Z0-9_]+)\)", value)
        if arg_search is not None:
            param_name = arg_search.group(1)
            return f"launch.substitutions.LaunchConfiguration('{param_name}'"
        # Check if it is an environment variable
        optenv_search = re.search(
            r"\$\((?:opt)?env ([a-zA-Z0-9_]+) ?([a-zA-Z0-9_]+)?\)",
            value
        )
        if optenv_search is not None:
            variable_name = optenv_search.group(1)
            default_value = optenv_search.group(2)
            if default_value is None:
                return f"os.environ.get('{variable_name}')"
            return f"os.environ.get('{variable_name}', '{default_value}')"
        # Check if it is a find package
        find_search = re.search(r"\$\(find ([a-zA-Z0-9_]+)\)(.*)", value)
        if find_search is not None:
            self.imports.update(
                [
                    "from launch.substitutions import PathJoinSubstitution",
                    "from launch_ros.substitutions import FindPackageShare"
                ]
            )
            package = find_search.group(1)
            file_path = self._convert_attribute(find_search.group(2))
            return f"PathJoinSubstitution([FindPackageShare('{package}'),\
                 '{file_path}''])"
        if re.search(r"\'", value):
            return f'"{value}"'
        return f"'{value}'"

    def _convert_node_elem(self, elem):
        """
        Convert a single <node> element to a launch action
        """
        launch_action = LaunchAction(
            "Node",
            {
                "package": self._renamed_get(elem, "pkg", None),
                "executable": self._renamed_get(elem, "type", None),
                "name": self._renamed_get(elem, "name", None),
                "output": self._renamed_get(elem, "output", None)
            })
        required_node = elem.attrib.get("required", None)
        if required_node is not None and required_node.lower() == "true":
            launch_action.set_parameter("on_exit", "launch.actions.Shutdown()")
        node_params = self.global_params + self._convert_param_elems(elem)
        if len(node_params) > 0:
            launch_action.set_parameter("parameters", node_params)
        if launch_action.parameters["package"][1:-1] not in KNOWN_ROS2_PKGS:
            problem_pkg = launch_action.parameters["package"]
            warn = f"A node in {problem_pkg} will be launched, but the " \
                " package could not be found with ament_cmake."
            self.problems.append(warn)
        return launch_action

    def _convert_param_elems(self, node_elem):
        """
        Convert all <param> elems attached to a node
        """
        parameters = []
        param_elems = node_elem.findall("param")
        for elem in param_elems:
            param_name = elem.attrib.get("name")
            elem_value = elem.attrib.get("value", None)
            param_value = self._convert_attribute(elem_value)
            parameter = {}
            parameter[param_name] = param_value
            parameters.append(parameter)
        return parameters

    def _convert_include_elem(self, elem):
        """
        Convert a sincle <include> element.
        """
        file_param = elem.attrib.get("file", None)
        if file_param is None:
            warn = "Found <include> elemenet without 'file' attribute"
            logging.warning(warn)
            self.problems.append(warn)
            self.problems.append(
                etree.tostring(elem, encoding="unicode").strip()
            )
            return None
        file_info = re.search(
            r"\$\(find ([a-zA-Z0-9_]+)\)(.*)",
            file_param
        )
        if file_info is None:
            warn = "Found <include> element with 'file' attribute that is" \
                   " invalid"
            logging.warning(warn)
            self.problems.append(warn)
            self.problems.append(
                etree.tostring(elem, encoding="unicode").strip()
            )
            return None
        package = file_info.group(1)
        path = file_info.group(2)
        arguments = {}
        for arg_elem in elem.findall("arg"):
            name = arg_elem.attrib.get("name")
            value = self._convert_attribute(arg_elem.attrib.get("default"))
            arguments[name] = value
        self.imports.update([
            "from ament_index_python.packages import"
            " get_package_share_directory",
            "import os.path"
        ])
        if package not in KNOWN_ROS2_PKGS:
            warn = f"Includes launch description from {package} but the " \
                "package was not found by ament_cmake."
            self.problems.append(warn)
        return IncludeLaunchDescription(package, path, arguments)

    def _convert_robot_description(self, elem):
        """
        Robot descriptions are converted to local arguments
        """
        # Check whether urdf or xacro
        urdf_attrib = elem.attrib.get("textfile", None)
        xacro_attrib = elem.attrib.get("command", None)
        # Things to do for either
        if urdf_attrib is not None or xacro_attrib is not None:
            self.imports.update(
                [
                    "from launch.substitutions import PathJoinSubstitution",
                    "from launch_ros.substitutions import FindPackageShare"
                ]
            )
            self.ld_entity_strings += [
                "robot_description",
                "robot_description_path"
            ]
        if urdf_attrib is not None:
            urdf_search = re.search(
                r"\$(find ([a-zA-Z0-9_]+)\)(.*)",
                urdf_attrib
            )
            package = urdf_search.group(1)
            subpath = urdf_search.group(2)
            urdf_location = "urdf_path = PathJoinSubstitution("
            urdf_location += f"[FindPackageShare('{package}'), '{subpath}'])"
            robot_description_path = "robot_description_path = " \
                                     "SetLaunchConfiguration(" \
                                     "name='robot_description_path'," \
                                     "value=urdf_path)"
            robot_description = "robot_description = SetLaunchConfiguration(" \
                                "name='robot_description'," \
                                "value=launch.substitutions.Command" \
                                "(['cat', ' ', urdf_path]))"
            self.preparatory.append(urdf_location)
            self.preparatory.append(robot_description)
            self.preparatory.append(robot_description_path)
            self.ld_entity_strings.append("robot_description")
            self.ld_entity_strings.append("robot_description_path")
        elif xacro_attrib is not None:
            find_search = re.search(
                r"\$\(find ([a-zA-Z0-9_]+)\)(.*)",
                xacro_attrib
            )
            if not find_search.group(1) == "xacro":
                warn = f"May have wrongly assumed robot_description" \
                       " was made with xacro."
                logging.warning(warn)
                self.problems.append(warn)
                self.problems.append(
                    etree.tostring(elem, encoding="unicode").strip()
                )
            # Modified search to remove final quotation mark
            xacro_search = re.search(
                r"\$\(find ([a-zA-Z0-9_]+)\)(.*)['\"]",
                find_search.group(2)
            )
            package = xacro_search.group(1)
            subpath = xacro_search.group(2)
            split_path = Path(subpath).parts
            xacro_path = "xacro_path = PathJoinSubstitution("
            xacro_path += f"[FindPackageShare('{package}'),"
            for part in split_path[:-1]:
                if part not in ["/", "\\"]:
                    xacro_path += "'" + part + "',"
            xacro_path += "'" + split_path[-1] + "'])"
            robot_description_path = "robot_description_path = launch." \
                                     "actions.SetLaunchConfiguration(" \
                                     "name='robot_description_path'," \
                                     "value=xacro_path)"
            robot_description = "robot_description = launch.actions." \
                                "SetLaunchConfiguration(" \
                                "name='robot_description'," \
                                "value=launch.substitutions.Command" \
                                "(['xacro', ' ', xacro_path]))"
            self.preparatory.append(xacro_path)
            self.preparatory.append(robot_description)
            self.preparatory.append(robot_description_path)

    def _generate_ld_strings(self, list_actions):
        """
        Generate the string for the Launch Description Entities
        """
        serialized_actions = []
        for action in list_actions:
            serialized_actions.append(action.serialize())
        return ",\n".join(self.ld_entity_strings + serialized_actions)

    def _renamed_get(self, elem, tag, default=None):
        """
        Get the element tag, but rename according to known new names.
        """
        if tag == "pkg":
            pkg_name = elem.attrib.get(tag, default)
            if pkg_name in RENAMED_ROS_PACKAGES.keys():
                new_pkg_name = RENAMED_ROS_PACKAGES[pkg_name]
                return self._convert_attribute(new_pkg_name)
        elif tag == "type":
            exe_name = elem.attrib.get(tag, default)
            if exe_name in RENAMED_ROS_EXECUTABLES.keys():
                new_exe_name = RENAMED_ROS_EXECUTABLES[exe_name]
                return self._convert_attribute(new_exe_name)
        return self._convert_attribute(elem.attrib.get(tag, default))

    def generate_report(self):
        if len(self.problems) > 0:
            print(self.problems)
            rep = f"Launch file: {self.src}\n"
            rep += "\t" + "\n\t".join(self.problems)
        else:
            rep = ""
        return rep


if __name__ == "__main__":
    #src = "/home/mathia/Programming/ros1_ws/src/motoman_gp25_support/launch/load_gp25.launch"
    src = "/home/mathia/Programming/ros1_ws/src/motoman_gp25_support/launch/robot_interface_streaming_gp25.launch"
    #src = "/home/mathia/Programming/ros1_ws/src/motoman_gp25_support/launch/robot_state_visualize_gp25.launch"
    conv = LaunchFilePorter(src)
    conv.port("./updated.launch")
