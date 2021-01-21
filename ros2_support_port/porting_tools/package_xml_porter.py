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

"""Contains a class and method for porting a package.xml file from catkin to ament"""
import xml.etree.ElementTree as etree
import xml.dom.minidom
import logging
from .constants import (RENAMED_ROS_PACKAGES,
                       NO_PKG_XML_DEPENDS,
                       PACKAGE_XML_ELEMENT_ORDER)

try:
    from ament_index_python import get_packages_with_prefixes
    KNOWN_ROS2_PKGS = get_packages_with_prefixes().keys()
except ImportError:
    KNOWN_ROS2_PKGS = []
    logging.info("Source a ROS2 workspace to get more info about missing \
        packages. Without, all packages will be listed as missing.")


def _new_element(tag, text="", attrib=None):
    """
    Helper function to make creating an element with a text and tail easier
    """
    if not attrib:
        attrib = {}
    element = etree.Element(tag, attrib=attrib)
    element.text = text
    return element


def _tag_order(tag):
    """
    Returns integer to order tags
    """
    if tag in PACKAGE_XML_ELEMENT_ORDER:
        return PACKAGE_XML_ELEMENT_ORDER.index(tag)
    return float("inf")


class PackageXMLPorter:
    """
    A class for porting a package.xml file from catkin to ament
    """
    def __init__(self, src):
        self.src = src
        with open(src, "r") as fsrc:
            self.contents = fsrc.read()
        self.found_packages = []
        self.renamed_packages = []
        self.removed_packages = []
        self.missing_packages = []

    def port(self, dst, extra_rules=[]):
        """
        Port the package xml file.
        """
        # Parse
        self.package_root = etree.fromstring(self.contents)
        self.tree = etree.ElementTree(self.package_root)

        # Rules to apply
        rules = [
            self.rule_set_format,
            self.rule_set_build_tool,
            self.rule_update_test_depend,
            self.rule_add_export_build_type,
            self.rule_set_run_to_exec_depend,
            self.rule_rename_packages,
            self.rule_remove_packages
        ]

        # Perform the rules on the contents
        for rule in rules + extra_rules:
            rule(self.package_root)

        # Reorder the elements
        self.package_root[:] = sorted(
            list(self.package_root),
            key=lambda elem: _tag_order(elem.tag)
        )

        # Add processing instructions and start prettyfying
        mdom = xml.dom.minidom.parseString(etree.tostring(self.package_root))
        mdom.insertBefore(
            mdom.createProcessingInstruction(
                "xml-model",
                'href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"'
            ),
            mdom.childNodes[0]
        )
        res = mdom.toprettyxml()
        # Remove the extra new lines that minidom makes
        res = "".join([s for s in res.splitlines(True) if s.strip()])
        # uhm... pretty enough I guess?
        with open(dst, "w") as fdst:
            fdst.write(res)

    #########################
    #        RULES          #
    #########################
    def rule_set_format(self, package_root):
        # ROS 2 supports formats 2, 3
        package_root.set("format", "3")

    def rule_set_build_tool(self, package_root):
        for elem in package_root.findall("buildtool_depend"):
            if elem.text and elem.text.strip() == "catkin":
                package_root.remove(elem)
        package_root.append(_new_element(
            tag="buildtool_depend",
            text="ament_cmake"))

    def rule_add_export_build_type(self, package_root):
        export_elem = package_root.find("export")
        if export_elem is not None:
            build_elem = export_elem.find("build_type")
            if build_elem is not None:
                build_elem.text = "ament_cmake"
            else:
                export_elem.append(
                    _new_element(
                        tag="build_type",
                        text="ament_cmake"
                    )
                )
        else:
            build_elem = _new_element(
                tag="build_type",
                text="ament_cmake"
            )
            export_elem = _new_element(tag="export", text="")
            export_elem.append(build_elem)
            package_root.append(export_elem)

    def rule_set_run_to_exec_depend(self, package_root):
        for elem in package_root.findall("run_depend"):
            elem.tag = "exec_depend"

    def rule_rename_packages(self, package_root):
        # Also identifies missing packages
        for elem in package_root.iter():
            if "depend" not in elem.tag or "_independent" in elem.tag:
                continue
            # General things
            self.found_packages.append(elem.text)
            if elem.text not in KNOWN_ROS2_PKGS:
                self.missing_packages.append(elem.text)
            # Rename packages
            if elem.text in RENAMED_ROS_PACKAGES.keys():
                new_name = RENAMED_ROS_PACKAGES[elem.text]
                self.renamed_packages.append(elem.text+" -> "+new_name)
                elem.text = new_name

    def rule_update_test_depend(self, package_root):
        package_root.append(
            _new_element(
                tag="test_depend",
                text="ament_lint_auto",
            )
        )
        package_root.append(
            _new_element(
                tag="test_depend",
                text="ament_lint_common"
            )
        )

    def rule_remove_packages(self, package_root):
        for elem in package_root.iter():
            if elem.text in NO_PKG_XML_DEPENDS:
                self.removed_packages.append(elem.text)
                package_root.remove(elem)

    #########################
    #          HELPERS      #
    #########################
    def generate_report(self):
        rep = ""
        rem_pkgs = len(self.removed_packages)
        mis_pkgs = len(self.missing_packages)
        ren_pkgs = len(self.renamed_packages)
        if rem_pkgs + mis_pkgs + ren_pkgs > 0:
            rep += f"Package.xml file: {self.src}\n"
            if mis_pkgs > 0:
                rep += "\nMissing packages:\n"
                rep += "\t" + "\n\t".join(self.missing_packages)
            if rem_pkgs > 0:
                rep += "\nRemoved packages:\n"
                rep += "\t" + "\n\t".join(self.removed_packages)
            if ren_pkgs > 0:
                rep += "\nRenamed packages:\n"
                rep += "\t" + "\n\t".join(self.renamed_packages)
        return rep


if __name__ == '__main__':
    src = "/home/mathia/Programming/ros1_ws/src/motoman_gp7_support/package.xml"
    conv = PackageXMLPorter(src)
    conv.port("./updated_package.xml")
